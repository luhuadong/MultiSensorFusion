#ifndef SRC_LIDAR_DRIVER_INTERNAL_H_
#define SRC_LIDAR_DRIVER_INTERNAL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <list>
#include <string>

#include <boost/function.hpp>

#include "point_types.h"
#include "input.h"

#include "pcap_reader.h"

#include "asensing_lidar/LidarScan.h"
#include "asensing_lidar/LidarPacket.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>

#define SOB_ANGLE_SIZE (4)
#define RAW_MEASURE_SIZE (3)
#define LASER_COUNT (40)
#define BLOCKS_PER_PACKET (10)
#define BLOCK_SIZE (RAW_MEASURE_SIZE * LASER_COUNT + SOB_ANGLE_SIZE)
#define TIMESTAMP_SIZE (4)
#define FACTORY_INFO_SIZE (1)
#define ECHO_SIZE (1)
#define RESERVE_SIZE (8)
#define REVOLUTION_SIZE (2)
#define INFO_SIZE (TIMESTAMP_SIZE + FACTORY_INFO_SIZE + ECHO_SIZE + RESERVE_SIZE + REVOLUTION_SIZE)
#define UTC_TIME (6)
#define PACKET_SIZE (BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE + UTC_TIME)
#define LASER_RETURN_TO_DISTANCE_RATE (0.004)
#define SEQ_NUM_SIZE (4)

/**
 * A0 LiDAR
 */
#define AG_LIDAR_A0_HEAD_SIZE       (40)
#define AG_LIDAR_TIME_SIZE          (6)
#define AG_LIDAR_A0_TIMESTAMP_SIZE  (4)
#define AG_LIDAR_A0_ECHO_SIZE       (1)
#define AG_LIDAR_A0_FACTORY_SIZE    (1)
#define AG_LIDAR_A0_RESERVED_SIZE   (8)
#define AG_LIDAR_A0_ENGINE_VELOCITY (2)

#define AG_LIDAR_A0_BLOCK_NUMBER (12)
#define AG_LIDAR_A0_BLOCK_HEADER (4)  /* each block first 4 byte */
#define AG_LIDAR_A0_UNIT_NUM     (8)  /* each block have 8 Unit */
#define AG_LIDAR_A0_UNIT_SIZE    (9)  /* each Unit have 9 byte */
#define AG_LIDAR_A0_BLOCK_SIZE   (AG_LIDAR_A0_UNIT_SIZE * AG_LIDAR_A0_UNIT_NUM + AG_LIDAR_A0_BLOCK_HEADER)
#define AG_LIDAR_A0_BLOCK_PACKET_BODY_SIZE (AG_LIDAR_A0_BLOCK_SIZE * AG_LIDAR_A0_BLOCK_NUMBER)

#define AG_LIDAR_A0_PACKET_TAIL_SIZE (4)

#define AG_LIDAR_A0_PACKET_SIZE (AG_LIDAR_A0_HEAD_SIZE + AG_LIDAR_A0_BLOCK_PACKET_BODY_SIZE + AG_LIDAR_A0_PACKET_TAIL_SIZE)

#define AGLidarSDK_DEFAULT_LIDAR_RECV_PORT 51180

#define MAX_LASER_NUM                   (256)
#define MAX_POINT_CLOUD_NUM             (1000000)
#define MAX_POINT_CLOUD_NUM_PER_CHANNEL (10000)
#define MAX_AZIMUTH_DEGREE_NUM          (36000)
#define COORDINATE_CORRECTION_CHECK     (false)

/************ A0 LiDAR Packet ************/
#pragma pack(push, 1)
typedef struct AG_LIDAR_A0_Header_s
{
    uint32_t Sob;     // 0xAA55A55A 4bytes
    uint32_t FrameID;

    uint16_t SeqNum;
    uint16_t PkgLen;
    uint16_t LidarType;
    uint8_t  VersionMajor;
    uint8_t  VersionMinor;

    uint8_t  UTCTime0;
    uint8_t  UTCTime1;
    uint8_t  UTCTime2;
    uint8_t  UTCTime3;
    uint8_t  UTCTime4;
    uint8_t  UTCTime5;
    uint32_t Timestamp;

    uint8_t  MeasureMode;

    uint8_t  LaserNum;
    uint8_t  BlockNum;
    uint8_t  EchoCount;
    uint8_t  TimeSyncMode;
    uint8_t  TimeSyncStat;
    uint8_t  MemsTemp;
    uint8_t  SlotNum;

    uint32_t PointNum;
    uint16_t Reserved;

public:
    AG_LIDAR_A0_Header_s() {
        Sob = 0;
        LaserNum = 0;
        BlockNum = 0;
    }
} AG_LIDAR_A0_Header;

typedef struct AG_LIDAR_A0_Unit_s
{
    uint16_t distance;      /* 球坐标系径向距离 radius（单位 mm） */
    uint16_t azimuth;       /* 球坐标系水平夹角，方位角（分辨率 0.01°） */
    uint16_t elevation;     /* 球坐标系垂直夹角，俯仰角/极角（分辨率 0.01°） */
    uint8_t  intensity;     /* 反射强度 intensity */
    uint16_t reserved;      /* 保留 */
} AG_LIDAR_A0_Unit;

typedef struct AG_LIDAR_A0_Block_s
{
    uint8_t channelNum;
    uint8_t timeOffSet;
    uint8_t returnSn;
    uint8_t reserved;
    AG_LIDAR_A0_Unit units[AG_LIDAR_A0_UNIT_NUM];
} AG_LIDAR_A0_Block;

typedef struct AG_LIDAR_A0_Tail_s
{
    uint8_t Reserved1;
    uint8_t Reserved2;
    uint8_t Reserved3;
    uint8_t Reserved4;
} AG_LIDAR_A0_Tail;

typedef struct AG_LIDAR_A0_Packet_s
{
    AG_LIDAR_A0_Header header;
    AG_LIDAR_A0_Block  blocks[AG_LIDAR_A0_BLOCK_NUMBER];
    AG_LIDAR_A0_Tail   tail;
    double timestamp_point;
} AG_LIDAR_A0_Packet;

#pragma pack(pop)
/*************** End of A0 ***************/

#define ROTATION_MAX_UNITS (36001)

typedef std::array<LidarPacket, 36000> PktArray;

typedef struct PacketsBuffer_s
{
  PktArray m_buffers{};
  PktArray::iterator m_iterPush;
  PktArray::iterator m_iterCalc;
  bool m_startFlag;

  inline PacketsBuffer_s() {
    m_iterPush = m_buffers.begin();//返回指向超始的迭代器
    m_iterCalc = m_buffers.begin();
    m_startFlag = false;
  }

  inline int push_back(LidarPacket pkt) {
    if (!m_startFlag) {
      *m_iterPush = pkt;//将当前迭代器指向的数组中该位置的值修改为pkt
      m_startFlag = true;
      return 1;
    } 
    m_iterPush++;

    if (m_iterPush == m_iterCalc) {
      printf("buffer don't have space!,%d\n", m_iterPush - m_buffers.begin());
      return 0;
    }

    if (m_buffers.end() == m_iterPush) {
      m_iterPush = m_buffers.begin();
      *m_iterPush = pkt;
    }
    *m_iterPush = pkt;
    return 1;
    
  }

  inline bool hasEnoughPackets() {
    return ((m_iterPush - m_iterCalc > 0 ) ||
            ((m_iterPush - m_iterCalc + 36000 > 0 ) && (m_buffers.end() - m_iterCalc < 1000) && (m_iterPush - m_buffers.begin() < 1000)));
  }

  inline PktArray::iterator getIterCalc() { return m_iterCalc;}
  inline void moveIterCalc() {
    m_iterCalc++;
    if (m_buffers.end() == m_iterCalc) {
      m_iterCalc = m_buffers.begin();
    }
  }
} PacketsBuffer;

class LidarDriver_Internal
{
 public:
  /**
   * @brief Constructor
   * @param device_ip  				The ip of the device
   *        lidar_port 				The port number of lidar data
   *        pcl_callback      The callback of PCL data structure
   *        type       				The device type
   */
  LidarDriver_Internal(
      std::string device_ip, uint16_t lidar_port,
      boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)>
          pcl_callback,
          uint16_t start_angle, int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType, // the default timestamp type is LiDAR time
          std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag,
          std::string target_frame, std::string fixed_frame);

  /**
   * @brief Constructor
   * @param pcap_path         The path of pcap file
   *        pcl_callback      The callback of PCL data structure
   *        start_angle       The start angle of frame
   *        tz                The timezone
   *        pcl_type          Structured Pointcloud
   *        frame_id          The frame id of pcd
   */
  LidarDriver_Internal(
      std::string pcap_path, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)> \
      pcl_callback, uint16_t start_angle, int tz, int pcl_type, \
      std::string lidar_type, std::string frame_id, std::string timestampType, // the default timestamp type is LiDAR time
      std::string lidar_correction_file, bool coordinate_correction_flag, \
      std::string target_frame, std::string fixed_frame);
  ~LidarDriver_Internal();

  /**
   * @brief load the correction file
   * @param correction The path of correction file
   */
  int LoadCorrectionFile(std::string correction);

  /**
   * @brief load the correction file
   * @param angle The start angle
   */
  void ResetStartAngle(uint16_t start_angle);

  void Start();
  void Stop();
  void PushScanPacket(asensing_lidar::LidarScanPtr scan);
  bool GetCorrectionFileFlag();
  void SetCorrectionFileFlag(bool flag);

 private:
  void Init();
  void RecvTask();
  void ProcessLiarPacket();
  void PushLiDARData(LidarPacket packet);

  // int ParseRawData(AG_LIDAR_A0_Packet *packet, const uint8_t *buf, const int len);
  int ParseA0Data(AG_LIDAR_A0_Packet *packet, const uint8_t *recvbuf, const int len);

  // void CalcPointXYZIT(AG_LIDAR_A0_Packet *pkt, int blockid, boost::shared_ptr<PPointCloud> cld);
  void CalcA0PointXYZIT(AG_LIDAR_A0_Packet *pkt, int blockid, char laserNumber, boost::shared_ptr<PPointCloud> cld);

  void FillPacket(const uint8_t *buf, const int len, double timestamp);

  void EmitBackMessege(char laserNumber, boost::shared_ptr<PPointCloud> cld, asensing_lidar::LidarScanPtr scan);
  void SetEnvironmentVariableTZ();
  asensing_lidar::LidarPacket SaveCorrectionFile(int laserNumber);

  bool calculateTransformMatrix(Eigen::Affine3f& matrix, const std::string& target_frame, const std::string& source_frame, const ros::Time& time);
  void manage_tf_buffer();
  bool computeTransformToTarget(const ros::Time &scan_time);
  bool computeTransformToFixed(const ros::Time &packet_time) ;
  void transformPoint(float& x, float& y, float& z);
  pthread_mutex_t lidar_lock_;//创建互斥锁
  sem_t lidar_sem_;//创建信号量对象
  boost::thread *lidar_recv_thr_;//构造一个表示当前执行线程的线程对象。雷达接收线程
  boost::thread *lidar_process_thr_;//雷达处理进程
  bool enable_lidar_recv_thr_;//雷达接收线程 使能
  bool enable_lidar_process_thr_;//雷达处理进程  使能
  int start_angle_;
  std::string m_sTimestampType;
  double m_dPktTimestamp;

  std::list<struct LidarPacket_s> lidar_packets_;

  boost::shared_ptr<Input> input_;//Input类型的指针（关于输入，如socket）
  boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp, asensing_lidar::LidarScanPtr scan)>
      pcl_callback_;

  float sin_lookup_table_[ROTATION_MAX_UNITS];
  float cos_lookup_table_[ROTATION_MAX_UNITS];

  uint16_t last_azimuth_;
  double last_timestamp_;

  float elev_angle_map_[LASER_COUNT];
  float horizatal_azimuth_offset_map_[LASER_COUNT];

  float General_elev_angle_map_[MAX_LASER_NUM];
  float General_horizatal_azimuth_offset_map_[MAX_LASER_NUM];

  float block40OffsetSingle_[BLOCKS_PER_PACKET];
  float block40OffsetDual_[BLOCKS_PER_PACKET];
  float laser40Offset_[LASER_COUNT];

  int tz_second_;
  std::string m_sFrameId;
  int pcl_type_;
  PcapReader *pcap_reader_;
  bool connect_lidar_;
  std::string m_sLidarType;
  std::vector<float> m_sin_azimuth_map_;//sin 水平方位角
  std::vector<float> m_cos_azimuth_map_;
  std::vector<float> m_sin_elevation_map_;//sin 垂直俯仰角
  std::vector<float> m_cos_elevation_map_;
  bool got_lidar_correction_flag;
  std::string correction_file_path_;
  PacketsBuffer m_PacketsBuffer;
  bool m_bCoordinateCorrectionFlag;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  Eigen::Affine3f m_tf_matrix_to_fixed;
  Eigen::Affine3f m_tf_matrix_to_target;
  std::string m_sSensorFrame;
  std::string m_sFixedFrame;
  std::string m_sTargetFrame;
  uint16_t m_iAzimuthRange;

};

#endif  /* SRC_LIDAR_DRIVER_INTERNAL_H_ */
