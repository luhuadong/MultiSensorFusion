#include <sstream>

#include "input.h"
#include "lidar_driver_internal.h"
#include "log.h"
#include <sched.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <algorithm>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double degreeToRadian(double degree) { return degree * M_PI / 180; }

// elevation angle of each line
static const float LidarGeneral_elev_angle_map[] = {
    14.882f, 11.032f, 8.059f, 5.057f, 3.04f, 2.028f, 1.86f, 1.688f, \
    1.522f, 1.351f, 1.184f, 1.013f, 0.846f, 0.675f, 0.508f, 0.337f, \
    0.169f, 0.0f, -0.169f, -0.337f, -0.508f, -0.675f, -0.845f, -1.013f, \
    -1.184f, -1.351f, -1.522f, -1.688f, -1.86f, -2.028f, -2.198f, -2.365f, \
    -2.536f, -2.7f, -2.873f, -3.04f, -3.21f, -3.375f, -3.548f, -3.712f, \
    -3.884f, -4.05f, -4.221f, -4.385f, -4.558f, -4.72f, -4.892f, -5.057f, \
    -5.229f, -5.391f, -5.565f, -5.726f, -5.898f, -6.061f, -7.063f, -8.059f, \
    -9.06f, -9.885f, -11.032f, -12.006f, -12.974f, -13.93f, -18.889f, -24.897f
};

// Lidar azimuth Horizatal offset
static const float LidarGeneral_horizatal_azimuth_offset_map[] = {
    -1.042f, -1.042f, -1.042f, -1.042f,  -1.042f, -1.042f, 1.042f, 3.125f, \
    5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, \
    -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, \
    1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f, \
    5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, \
    -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, \
    1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, -1.042f, -1.042f, \
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f
};

static std::vector<std::vector<PPoint> > PointCloudList(MAX_LASER_NUM);
// static std::vector<PPoint> PointCloud;
static std::vector<PPoint> PointCloud(MAX_POINT_CLOUD_NUM);
static int iPointCloudIndex = 0;

LidarDriver_Internal::LidarDriver_Internal(
    std::string device_ip, uint16_t lidar_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)> pcl_callback,
    uint16_t start_angle, int tz,
    int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,
    std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag,
    std::string target_frame, std::string fixed_frame) {
      // LOG_FUNC();
  pthread_mutex_init(&lidar_lock_, NULL);//C函数的多线程编程中，互斥锁的初始化
  sem_init(&lidar_sem_, 0, 0);//创建信号量，并为信号量值赋初值
    //int sem_init(sem_t *sem, int pshared, unsigned int value);
    //sem ：指向信号量对象
    //pshared : 指明信号量的类型。当不为0时，用于进程；当为0时，用于线程。
    //value : 指定信号量值的大小
    //返回值：成功返回0，失败时返回-1，并设置errno。

  lidar_recv_thr_ = NULL;//雷达接收线程指针
  lidar_process_thr_ = NULL;//雷达处理线程指针

  enable_lidar_recv_thr_ = false;//使能关闭
  enable_lidar_process_thr_ = false;

  input_.reset(new Input(lidar_port, multicast_ip)); //p.reset(q) 会令智能指针p中存放指针q，即p指向q的空间，而且会释放原来的空间。（默认是delete）

  start_angle_ = start_angle;
  pcl_callback_ = pcl_callback;
  last_azimuth_ = 0;
  last_timestamp_ = 0;
  m_sLidarType = lidar_type;
  m_sSensorFrame = frame_id;
  m_sFixedFrame = fixed_frame;
  m_sTargetFrame = target_frame;
  m_iAzimuthRange = MAX_AZIMUTH_DEGREE_NUM;
  if (!m_sTargetFrame.empty())
  {
    m_sFrameId = m_sTargetFrame;
  }
  else if (!m_sFixedFrame.empty())
  {
    m_sFrameId = m_sFixedFrame;
  }
  else
  {
    m_sFrameId = m_sSensorFrame;
  }

  tz_second_ = tz * 3600;
  pcl_type_ = pcl_type;
  connect_lidar_ = true;
  pcap_reader_ = NULL;
  m_sTimestampType = timestampType;
  m_dPktTimestamp = 0.0f;
  got_lidar_correction_flag = false;
  correction_file_path_ = lidar_correction_file;
  m_bCoordinateCorrectionFlag = coordinate_correction_flag;
  Init();
}

LidarDriver_Internal::LidarDriver_Internal(std::string pcap_path, \
    boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)> \
    pcl_callback, uint16_t start_angle, int tz, int pcl_type, \
    std::string lidar_type, std::string frame_id, std::string timestampType,
    std::string lidar_correction_file, bool coordinate_correction_flag,
    std::string target_frame, std::string fixed_frame) {
  pthread_mutex_init(&lidar_lock_, NULL);
  sem_init(&lidar_sem_, 0, 0);

  lidar_recv_thr_ = NULL;
  lidar_process_thr_ = NULL;

  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;

  pcap_reader_ = new PcapReader(pcap_path,lidar_type);

  start_angle_ = start_angle;
  pcl_callback_ = pcl_callback;
  last_azimuth_ = 0;
  last_timestamp_ = 0;
  m_sLidarType = lidar_type;
   m_sSensorFrame = frame_id;
  m_sFixedFrame = fixed_frame;
  m_sTargetFrame = target_frame;
  m_iAzimuthRange = MAX_AZIMUTH_DEGREE_NUM;
  if (!m_sTargetFrame.empty())
  {
    m_sFrameId = m_sTargetFrame;
  }
  else if (!m_sFixedFrame.empty())
  {
    m_sFrameId = m_sFixedFrame;
  }
  else
  {
    m_sFrameId = m_sSensorFrame;
  }
  tz_second_ = tz * 3600;
  pcl_type_ = pcl_type;
  connect_lidar_ = false;
  m_sTimestampType = timestampType;
  m_dPktTimestamp = 0.0f;
  got_lidar_correction_flag = false;
  correction_file_path_ = lidar_correction_file;
  m_bCoordinateCorrectionFlag = coordinate_correction_flag;
  Init();
}

LidarDriver_Internal::~LidarDriver_Internal() {
  Stop();
  sem_destroy(&lidar_sem_);
  pthread_mutex_destroy(&lidar_lock_);

  if (pcap_reader_ != NULL) {
    delete pcap_reader_;
    pcap_reader_ = NULL;
  }
}

void LidarDriver_Internal::Init()
{
  printf("Packet size: %d, %d\n", AG_LIDAR_A0_PACKET_SIZE, sizeof(AG_LIDAR_A0_Packet));

  for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex) {
    float rotation = degreeToRadian(0.01 * static_cast<double>(rotIndex));//旋转角度   转弧度 
    cos_lookup_table_[rotIndex] = cosf(rotation);//cosf求弧度值的余弦值
    sin_lookup_table_[rotIndex] = sinf(rotation);
  }
  m_sin_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);//resize函数可以用于vector数组的初始化 调整容器m_sin_a...的大小为MAX_AZI...，扩容后的每个元素的值为element，默认为0
  m_cos_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  for(int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    m_sin_azimuth_map_[i] = sinf(i * M_PI / 18000);
    m_cos_azimuth_map_[i] = cosf(i * M_PI / 18000);
  }
  
  if (pcl_type_) {
    for (int i = 0; i < MAX_LASER_NUM; i++) {
      PointCloudList[i].reserve(MAX_POINT_CLOUD_NUM_PER_CHANNEL);
    }
  }

  if (m_sLidarType == "A0") {
    m_sin_elevation_map_.resize(AG_LIDAR_A0_UNIT_NUM); // LASER_COUNT
    m_cos_elevation_map_.resize(AG_LIDAR_A0_UNIT_NUM);
    for (int i = 0; i < AG_LIDAR_A0_UNIT_NUM; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(LidarGeneral_elev_angle_map[i]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(LidarGeneral_elev_angle_map[i]));
      General_elev_angle_map_[i] = LidarGeneral_elev_angle_map[i];
      General_horizatal_azimuth_offset_map_[i] = \
          LidarGeneral_horizatal_azimuth_offset_map[i];//
    }
  }

  SetEnvironmentVariableTZ();
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int LidarDriver_Internal::LoadCorrectionFile(std::string correction_content)
{
  // LOG_FUNC();
  // LOG_D("stirng:[%s]",correction_content.c_str());
  std::istringstream ifs(correction_content);

  std::string line;
  if (std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
    std::cout << "Parse Lidar Correction..." << std::endl;
  }

  double azimuthOffset[AG_LIDAR_A0_UNIT_NUM];
  double elev_angle[AG_LIDAR_A0_UNIT_NUM];

  int lineCounter = 0;
  while (std::getline(ifs, line)) {
    // correction file has 3 columns, min length is len(0,0,0)
    if (line.length() < 5) {
      break;
    }
    lineCounter++;

    int lineId = 0;
    double elev, azimuth;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> lineId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elev;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;

    if (lineId != lineCounter) {
      break;
    }

    elev_angle[lineId - 1] = elev;
    azimuthOffset[lineId - 1] = azimuth;
  }
  m_sin_elevation_map_.resize(lineCounter);
  m_cos_elevation_map_.resize(lineCounter);
  for (int i = 0; i < lineCounter; ++i) {
    /* for all the laser offset */
    General_elev_angle_map_[i] = elev_angle[i];
    m_sin_elevation_map_[i] = sinf(degreeToRadian(General_elev_angle_map_[i]));
    m_cos_elevation_map_[i] = cosf(degreeToRadian(General_elev_angle_map_[i]));
    General_horizatal_azimuth_offset_map_[i] = azimuthOffset[i];
  }

  return 0;
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void LidarDriver_Internal::ResetStartAngle(uint16_t start_angle)
{
  start_angle_ = start_angle;
}

void LidarDriver_Internal::Start()
{
  // LOG_FUNC();
  Stop();
  enable_lidar_recv_thr_ = true;
  enable_lidar_process_thr_ = true;
  lidar_process_thr_ = new boost::thread(
      boost::bind(&LidarDriver_Internal::ProcessLiarPacket, this));

  if (connect_lidar_) {
    lidar_recv_thr_ =
        new boost::thread(boost::bind(&LidarDriver_Internal::RecvTask, this));
  } else {
    pcap_reader_->start(boost::bind(&LidarDriver_Internal::FillPacket, this, _1, _2, _3));
  }
}

void LidarDriver_Internal::Stop()
{
  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;

  if (lidar_process_thr_) {
    lidar_process_thr_->join();
    delete lidar_process_thr_;
    lidar_process_thr_ = NULL;
  }

  if (lidar_recv_thr_) {
    lidar_recv_thr_->join();
    delete lidar_recv_thr_;
    lidar_recv_thr_ = NULL;
  }

  if (pcap_reader_ != NULL) {
    pcap_reader_->stop();
  }

  return;
}

void LidarDriver_Internal::RecvTask()
{
  // LOG_FUNC();
  int ret = 0;
  sched_param param;
  int ret_policy;
  // SCHED_FIFO和SCHED_RR
  param.sched_priority = 99;
  int rc = pthread_setschedparam(pthread_self(), SCHED_RR, &param);
  printf("publishRawDataThread:set result [%d]\n", rc);
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  printf("publishRawDataThread:get thead %lu, policy %d and priority %d\n", pthread_self(), ret_policy, param.sched_priority);

  while (enable_lidar_recv_thr_) {

    LidarPacket pkt;
    int rc = input_->getPacket(&pkt);
    if (rc == -1) {
      continue;
    }

    PushLiDARData(pkt);
  }
}

void LidarDriver_Internal::FillPacket(const uint8_t *buf, const int len, double timestamp)
{
    LidarPacket pkt;
    memcpy(pkt.data, buf, len);
    pkt.size = len;
    pkt.stamp = timestamp;
    PushLiDARData(pkt);
}

void LidarDriver_Internal::ProcessLiarPacket()
{
  // LOG_FUNC();
  double lastTimestamp = 0.0f;
  struct timespec ts;
  int ret = 0;

  boost::shared_ptr<PPointCloud> outMsg(new PPointCloud());
  asensing_lidar::LidarScanPtr scan(new asensing_lidar::LidarScan);
  asensing_lidar::LidarPacket rawpacket;
  if(!computeTransformToTarget(ros::Time::now()))
    return;

  while (enable_lidar_process_thr_) {
    if (!m_PacketsBuffer.hasEnoughPackets()) {
      usleep(1000);
      continue;
    }
    LidarPacket packet = *(m_PacketsBuffer.getIterCalc());//a=*b,把b指针指向的地址里的值赋给a，即将迭代器m_iterCalc的当前pkt赋给packet
    m_PacketsBuffer.moveIterCalc();//让迭代器m_iterCalc移到下一个pkt
    rawpacket.stamp.sec = floor(packet.stamp);
    rawpacket.stamp.nsec = (packet.stamp - floor(packet.stamp))*1000000000;
    rawpacket.size = packet.size;
    rawpacket.data.resize(packet.size);
    memcpy(&rawpacket.data[0], &packet.data[0], packet.size);
    m_dPktTimestamp = packet.stamp;
    // printf("##m_dPktTimestamp: %lf\n", m_dPktTimestamp);

    if (packet.size == AG_LIDAR_A0_PACKET_SIZE) {
      manage_tf_buffer();
      if(!computeTransformToFixed(rawpacket.stamp))
        return; // target frame not available
      AG_LIDAR_A0_Packet pkt;
      ret = ParseA0Data(&pkt, packet.data, packet.size);

      if (ret != 0) {
        continue;
      }
      //scan->packets.push_back(rawpacket);
      for (int i = 0; i < pkt.header.BlockNum; ++i) {
        int azimuthGap = 0; // To do 
        double timestampGap = 0; // To do 
        if(last_azimuth_ > pkt.blocks[i].units[0].azimuth) {
          azimuthGap = static_cast<int>(pkt.blocks[i].units[0].azimuth) + (36000 - static_cast<int>(last_azimuth_));
        } else {
          azimuthGap = static_cast<int>(pkt.blocks[i].units[0].azimuth) - static_cast<int>(last_azimuth_);
        }
        timestampGap = pkt.timestamp_point - last_timestamp_ + 0.001;
        
        if (last_azimuth_ != pkt.blocks[i].units[0].azimuth && 
                      (azimuthGap / timestampGap) < MAX_AZIMUTH_DEGREE_NUM * 100 ) {
          /* for all the blocks */
          if ((last_azimuth_ > pkt.blocks[i].units[0].azimuth &&
               start_angle_ <= pkt.blocks[i].units[0].azimuth) ||
              (last_azimuth_ < start_angle_ &&
               start_angle_ <= pkt.blocks[i].units[0].azimuth)) {
            //if (pcl_callback_ && (iPointCloudIndex > 0 || PointCloudList[0].size() > 0)) {
              //scan->packets.push_back(SaveCorrectionFile(pkt.header.LaserNum));
              EmitBackMessege(pkt.header.LaserNum, outMsg, scan);
              //scan->packets.clear();
              if(!computeTransformToTarget(rawpacket.stamp))
                return; // target frame not available
            //}
          }
        } else {
          //printf("last_azimuth_:%d pkt.blocks[i].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[i].azimuth, azimuthGap);
        }

        CalcA0PointXYZIT(&pkt, i, pkt.header.LaserNum, outMsg);
        last_azimuth_ = pkt.blocks[i].units[0].azimuth;
        last_timestamp_ = pkt.timestamp_point;
      }
    }
    else {
      continue;
    }

    outMsg->header.frame_id = m_sFrameId;
    outMsg->height = 1;
  }
}

void LidarDriver_Internal::PushLiDARData(LidarPacket packet) {
  m_PacketsBuffer.push_back(packet);
}

int LidarDriver_Internal::ParseA0Data(AG_LIDAR_A0_Packet *packet, const uint8_t *recvbuf, const int len)
{
  if (len != AG_LIDAR_A0_PACKET_SIZE) {
    std::cout << "packet size mismatch LidarDriver_Internal " << len << "," << \
        len << std::endl;
    return -1;
  }

  int index = 0;
  int block = 0;
  //Parse 8 Bytes Header
  packet->header.Sob = (recvbuf[index] & 0xff) << 24| (recvbuf[index +1] & 0xff) << 16| (recvbuf[index +2] & 0xff) << 8| ((recvbuf[index+3] & 0xff));
  packet->header.FrameID = (recvbuf[index+4] & 0xff) << 24| (recvbuf[index +5] & 0xff) << 16| (recvbuf[index +6] & 0xff) << 8| ((recvbuf[index+7] & 0xff));

  packet->header.SeqNum = (recvbuf[index +8]& 0xff) | ((recvbuf[index + 9]& 0xff) << 8);
  packet->header.PkgLen = (recvbuf[index +10]& 0xff) | ((recvbuf[index + 11]& 0xff) << 8);
  packet->header.LidarType = (recvbuf[index +12]& 0xff) | ((recvbuf[index + 13]& 0xff) << 8);
  packet->header.VersionMajor = recvbuf[index+14] & 0xff;
  packet->header.VersionMinor = recvbuf[index+15] & 0xff;

  packet->header.UTCTime0 = recvbuf[index+16] & 0xff;
  packet->header.UTCTime1 = recvbuf[index+17] & 0xff;
  packet->header.UTCTime2 = recvbuf[index+18] & 0xff;
  packet->header.UTCTime3 = recvbuf[index+19] & 0xff;
  packet->header.UTCTime4 = recvbuf[index+20] & 0xff;
  packet->header.UTCTime5 = recvbuf[index+21] & 0xff;
  packet->header.Timestamp = (recvbuf[index +22] & 0xff)| (recvbuf[index+23] & 0xff) << 8 | \
      ((recvbuf[index+24] & 0xff) << 16) | ((recvbuf[index+25] & 0xff) << 24);

  packet->header.MeasureMode = recvbuf[index+26] & 0xff;
  packet->header.LaserNum = recvbuf[index+27] & 0xff;
  packet->header.BlockNum = recvbuf[index+28] & 0xff;

  packet->header.EchoCount = recvbuf[index+29] & 0xff;
  packet->header.TimeSyncMode = recvbuf[index+30] & 0xff;
  packet->header.TimeSyncStat = recvbuf[index+31] & 0xff;
  packet->header.MemsTemp = recvbuf[index+32] & 0xff;
  packet->header.SlotNum = recvbuf[index+33] & 0xff;

  index += AG_LIDAR_A0_HEAD_SIZE;

#if 0
  if (packet->header.Sob != htole32(0xAA55A55A)) {
    printf("Error Start of Packet!\n");
    return -1;
  }
#endif

  for(block = 0; block < packet->header.BlockNum; block++) {
    packet->blocks[block].channelNum = (recvbuf[index ]& 0xff);
    packet->blocks[block].timeOffSet = (recvbuf[index+1]& 0xff);
    packet->blocks[block].returnSn = (recvbuf[index+2]& 0xff);
    packet->blocks[block].reserved = (recvbuf[index+3]& 0xff);
    index += AG_LIDAR_A0_BLOCK_HEADER;

    int unit;

    for(unit = 0; unit < packet->header.LaserNum; unit++) {
      packet->blocks[block].units[unit].distance  = recvbuf[index + 0] | (recvbuf[index + 1] << 8);
      packet->blocks[block].units[unit].azimuth   = recvbuf[index + 2] | (recvbuf[index + 3] << 8);
      packet->blocks[block].units[unit].elevation = recvbuf[index + 4] | (recvbuf[index + 5] << 8);
      packet->blocks[block].units[unit].intensity = recvbuf[index + 6];
      
      index += AG_LIDAR_A0_UNIT_SIZE;
    }
  }

  index +=4;
  
  struct tm t = {0};

  t.tm_year = packet->header.UTCTime0;
  t.tm_mon  = packet->header.UTCTime1 - 1;
  t.tm_mday = packet->header.UTCTime2;
  t.tm_hour = packet->header.UTCTime3;
  t.tm_min  = packet->header.UTCTime4;
  t.tm_sec  = packet->header.UTCTime5;

  t.tm_isdst = 0;
  packet->timestamp_point = mktime(&t) + static_cast<double>(packet->header.Timestamp) / 1000000.0;
  return 0;
}

void LidarDriver_Internal::CalcA0PointXYZIT(AG_LIDAR_A0_Packet *pkt, int blockid, \
                                              char laserNumber, boost::shared_ptr<PPointCloud> cld) 
{
  AG_LIDAR_A0_Block *block = &pkt->blocks[blockid];

  for (int i = 0; i < laserNumber; ++i) {
    /* for all the units in a block */
    AG_LIDAR_A0_Unit &unit = block->units[i];
    PPoint point;

    float distance = static_cast<float>(unit.distance) / 100;

    /* skip wrong points */
    if (distance <= 0.1 || distance > 200.0) {
      continue;
    }

    //int azimuth = static_cast<int>(General_horizatal_azimuth_offset_map_[i] * 100 + block->units[0].azimuth);
    float azimuth = static_cast<float>(unit.azimuth) / 100;
    float elevation = static_cast<float>(unit.elevation) / 100;

    if(elevation < 0) {
      elevation += 360.0f;
    }
    else if(elevation >= 360.0f) {
      elevation -= 360.0f;
    }

    //float xyDistance = distance * m_cos_elevation_map_[i];
    float xyDistance = distance * cos(elevation * (3.1415926/180));

    point.x = static_cast<float>(xyDistance * sin(azimuth * (3.1415926/180)));
    point.y = static_cast<float>(xyDistance * cos(azimuth * (3.1415926/180)));
    point.z = static_cast<float>(distance * sin(elevation * (3.1415926/180)));
    transformPoint(point.x, point.y, point.z);  

    point.intensity = unit.intensity;

    if ("realtime" == m_sTimestampType) {
      point.timestamp = m_dPktTimestamp;
    }
    else {
      point.timestamp = pkt->timestamp_point + tz_second_ + static_cast<double>(pkt->blocks->timeOffSet) / 1000000.0f;
    }

    point.ring = i;

    if (pcl_type_) {
      PointCloudList[i].push_back(point);
    } else {
      PointCloud[iPointCloudIndex] = point;
      iPointCloudIndex++;
    }
  }
}

void LidarDriver_Internal::EmitBackMessege(char laserNumber, boost::shared_ptr<PPointCloud> cld, asensing_lidar::LidarScanPtr scan)
{
  if (pcl_type_) {
    for (int i=0; i<laserNumber; i++) {
      for (int j=0; j<PointCloudList[i].size(); j++) {
        cld->push_back(PointCloudList[i][j]);
      }
    }
  }
  else{
    cld->points.assign(PointCloud.begin(),PointCloud.begin() + iPointCloudIndex);
    cld->width = (uint32_t)cld->points.size();
    cld->height = 1;
    iPointCloudIndex = 0;
  }
  pcl_callback_(cld, cld->points[0].timestamp, scan); // the timestamp from first point cloud of cld
  if (pcl_type_) {
    for (int i=0; i<laserNumber; i++) {
      PointCloudList[i].clear();
      PointCloudList[i].reserve(MAX_POINT_CLOUD_NUM_PER_CHANNEL);
      cld->points.clear();
      cld->width = (uint32_t)cld->points.size();
      cld->height = 1;
    }
  }
}

void LidarDriver_Internal::PushScanPacket(asensing_lidar::LidarScanPtr scan)
{
  for(int i = 0; i < scan->packets.size(); i++) {
    if (scan->packets[i].data[0] == 0x47 && scan->packets[i].data[1] == 0x74){  //correction file
      if (got_lidar_correction_flag){
        continue;
      }
      else{
        std::cout << "Load correction file from rosbag" << std::endl;
        int correction_lenth = ((scan->packets[i].data[4] & 0xff) << 24) | ((scan->packets[i].data[5] & 0xff) << 16) | 
                              ((scan->packets[i].data[6] & 0xff) << 8) | ((scan->packets[i].data[7] & 0xff) << 0);
        if (correction_lenth == scan->packets[i].size){
          char buffer[correction_lenth];
          memcpy(buffer, &(scan->packets[i].data[8]), scan->packets[i].size);
          std::string correction_string = std::string(buffer);
          int ret = LoadCorrectionFile(correction_string);
          if (ret != 0) {
            std::cout << "Load correction file from rosbag failed" << std::endl;
          } 
          else {
            std::cout << "Load correction file from rosbag succeed" << std::endl;
            got_lidar_correction_flag = true;
          }
        }
        else{
          printf("Load correction file from rosbag failed");
        } 
        if(!got_lidar_correction_flag){
          std::ifstream fin(correction_file_path_);
          if (fin.is_open()) {
            std::cout << "Open correction file " << correction_file_path_ << " succeed" << std::endl;
          }
          else{
            std::cout << "Open correction file " << correction_file_path_ <<" failed" << std::endl;
            got_lidar_correction_flag = true;
            continue;
          }
          int length = 0;
          std::string strlidarCalibration;
          fin.seekg(0, std::ios::end);
          length = fin.tellg();
          fin.seekg(0, std::ios::beg);
          char *buffer = new char[length];
          fin.read(buffer, length);
          fin.close();
          strlidarCalibration = buffer;
          int ret = LoadCorrectionFile(strlidarCalibration);
          if (ret != 0) {
            std::cout << "Load correction file from " << correction_file_path_ <<" failed" << std::endl;
          } else {
            std::cout << "Load correction file from " << correction_file_path_ << " succeed" << std::endl;
          }
          got_lidar_correction_flag = true;
        }

      }
    }
    else {                                                                  //pcap
      LidarPacket pkt;
      pkt.stamp = scan->packets[i].stamp.sec + scan->packets[i].stamp.nsec / 1000000000.0;
      pkt.size = scan->packets[i].size;
      memcpy(&pkt.data[0], &(scan->packets[i].data[0]), scan->packets[i].size);
      PushLiDARData(pkt);
    }
  }
}

//设置环境变量TZ
void LidarDriver_Internal::SetEnvironmentVariableTZ()
{
  char *TZ; 
  
  //getenv()用来取得参数name环境变量的内容
  if((TZ = getenv("TZ")))
  {
    printf("TZ=%s\n",TZ); 
    return;
  } 
  unsigned int timezone = 0;
  time_t t1, t2 ;
  struct tm *tm_local, *tm_utc;
  time(&t1);
  t2 = t1;
  tm_local = localtime(&t1);
  t1 = mktime(tm_local) ;
  tm_utc = gmtime(&t2);
  t2 = mktime(tm_utc);
  timezone = 0;
  std::string data = "TZ=UTC" + std::to_string(timezone);
  int len = data.length();
  TZ = (char *)malloc((len + 1) * sizeof(char));
  data.copy(TZ, len, 0); 
  if(putenv(TZ) == 0){
    printf("set environment %s\n", TZ);
  }
  else{
    printf("set environment fail\n");
  }
}

asensing_lidar::LidarPacket LidarDriver_Internal::SaveCorrectionFile(int laserNumber)
{
  asensing_lidar::LidarPacket result;
  std::stringstream content;
  content<< "Laser id,Elevation,Azimuth" << std::endl;
  for(int i = 0; i < laserNumber; i++){
    content<< (i + 1) << "," << General_elev_angle_map_[i] << "," << General_horizatal_azimuth_offset_map_[i] << std::endl;
  }
  int length = content.str().size();
  result.size = length;
  result.data.resize(length + 8);
  result.data[0] = 0x47;
  result.data[1] = 0x74;
  result.data[2] = 0x05;
  result.data[3] = 0x00;
  result.data[4] = (length >> 24) & 0xff;
  result.data[5] = (length >> 16) & 0xff;
  result.data[6] = (length >> 8) & 0xff;
  result.data[7] = (length) & 0xff;
  memcpy(&result.data[8], content.str().c_str(), length);
  return result;
}

bool LidarDriver_Internal::GetCorrectionFileFlag()
{
  return got_lidar_correction_flag;
}

void LidarDriver_Internal::SetCorrectionFileFlag(bool flag )
{
  got_lidar_correction_flag = flag;
}


void LidarDriver_Internal::manage_tf_buffer()
{
    // check if sensor frame is already known, if not don't prepare tf buffer until setup was called
    if ( m_sSensorFrame.empty())
    {
      return;
    }

    // avoid doing transformation when  m_sSensorFrame equals target frame and no ego motion compensation is perfomed
    if (m_sFixedFrame.empty() &&  m_sSensorFrame == m_sTargetFrame)
    {
      // when the string is empty the points will not be transformed later on
      m_sTargetFrame = "";
      return;
    }

    // only use somewhat resource intensive tf listener when transformations are necessary
    if (!m_sFixedFrame.empty() || !m_sTargetFrame.empty())
    {
      if (!m_tf_buffer)
      {
        m_tf_buffer = std::make_shared<tf2_ros::Buffer>();
        m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
      }
    }
    else
    {
      m_tf_listener.reset();
      m_tf_buffer.reset();
    }
  }

  bool LidarDriver_Internal::calculateTransformMatrix(Eigen::Affine3f& matrix, const std::string& target_frame,
                                       const std::string& source_frame, const ros::Time& time)
  {
    if (!m_tf_buffer)
    {
      ROS_ERROR("tf buffer was not initialized yet");
      return false;
    }

    geometry_msgs::TransformStamped msg;
    try
    {
      msg = m_tf_buffer->lookupTransform(target_frame, source_frame, time, ros::Duration(0.2));
    }
    catch (tf2::LookupException& e)
    {
      ROS_ERROR("%s", e.what());
      return false;
    }
    catch (tf2::ExtrapolationException& e)
    {
      ROS_ERROR("%s", e.what());
      return false;
    }

    const geometry_msgs::Quaternion& quaternion = msg.transform.rotation;
    Eigen::Quaternionf rotation(quaternion.w, quaternion.x, quaternion.y, quaternion.z);

    const geometry_msgs::Vector3& origin = msg.transform.translation;
    Eigen::Translation3f translation(origin.x, origin.y, origin.z);

    matrix = translation * rotation;
    return true;
  }

  bool LidarDriver_Internal::computeTransformToTarget(const ros::Time &scan_time)
  {
    if (m_sTargetFrame.empty())
    {
      // no need to calculate transform -> success
      return true;
    }
    std::string& source_frame = m_sFixedFrame.empty() ?  m_sSensorFrame : m_sFixedFrame;
    return calculateTransformMatrix(m_tf_matrix_to_target, m_sTargetFrame, source_frame, scan_time);
  }

  bool LidarDriver_Internal::computeTransformToFixed(const ros::Time &packet_time)
  {
    if (m_sFixedFrame.empty())
    {
      // no need to calculate transform -> success
      return true;
    }
    std::string &source_frame =  m_sSensorFrame;
    return calculateTransformMatrix(m_tf_matrix_to_fixed, m_sFixedFrame, source_frame, packet_time);
  }

  void LidarDriver_Internal::transformPoint(float& x, float& y, float& z)
  {
    if(m_sFixedFrame.empty() && m_sTargetFrame.empty())
      return;
    Eigen::Vector3f p = Eigen::Vector3f(x, y, z);
    if (!m_sFixedFrame.empty())
    {
      p = m_tf_matrix_to_fixed * p;
    }
    if (!m_sTargetFrame.empty())
    {
      p = m_tf_matrix_to_target * p;
    }
    x = p.x();
    y = p.y();
    z = p.z();
  }
