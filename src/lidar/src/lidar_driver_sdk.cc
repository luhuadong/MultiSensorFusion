#include "lidar_driver_sdk.h"
#include "src/tcp_command_client.h"
#include "yaml-cpp/yaml.h"
#include "log.h"
#include "version.h"
#include <fstream>

#define TCP_COMMAND_PORT    (9347)

LidarDriverSDK::LidarDriverSDK( // 连接雷达的
    std::string device_ip, const uint16_t lidar_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)>
        pcl_callback,
    uint16_t start_angle,
    int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,//tz表示时区
    std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag,
    std::string target_frame, std::string fixed_frame) {
  printVersion();     /* 打印版本 */
  lidarDriver = NULL; /* 指针先指空 */
  // LOG_FUNC();

  lidarDriver = new LidarDriver(device_ip, lidar_port,
            pcl_callback, start_angle, tz, pcl_type, lidar_type, frame_id, timestampType, \
            lidar_correction_file, multicast_ip, coordinate_correction_flag, target_frame, fixed_frame);

  tcp_command_client_ =
      TcpCommandClientNew(device_ip.c_str(), TCP_COMMAND_PORT);//通过STRING类对象的成员函数c_str()把string对象转换为c中字符串的样式
  if (!tcp_command_client_) {
    std::cout << "Init TCP Command Client Failed" << std::endl;
  }
  get_calibration_thr_ = NULL;
  enable_get_calibration_thr_ = false;
  got_lidar_calibration_ = false;
  correction_file_path_ = lidar_correction_file;
}

LidarDriverSDK::LidarDriverSDK(\
    std::string pcap_path, \
    boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)> pcl_callback, \
    uint16_t start_angle, int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,
    std::string lidar_correction_file, bool coordinate_correction_flag,
    std::string target_frame, std::string fixed_frame) {
  printVersion();
  lidarDriver = NULL;

  lidarDriver = new LidarDriver(pcap_path, pcl_callback, start_angle, \
      tz, pcl_type, lidar_type, frame_id, timestampType, lidar_correction_file, \
      coordinate_correction_flag, target_frame, fixed_frame);

  get_calibration_thr_ = NULL;
  tcp_command_client_ = NULL;
  enable_get_calibration_thr_ = false;
  got_lidar_calibration_ = false;
  correction_file_path_ = lidar_correction_file;
}

LidarDriverSDK::~LidarDriverSDK() {
  Stop();
  if (lidarDriver) {
    delete lidarDriver;
  }
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int LidarDriverSDK::LoadLidarCorrectionFile(
    std::string correction_content) {
  return lidarDriver->LoadCorrectionFile(correction_content);
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void LidarDriverSDK::ResetLidarStartAngle(uint16_t start_angle) {
  if (!lidarDriver) return;
  lidarDriver->ResetStartAngle(start_angle);
}

std::string LidarDriverSDK::GetLidarCalibration() {
  return correction_content_;
}

void LidarDriverSDK::Start() {
// LOG_FUNC();
  Stop();

  if (lidarDriver) {
    lidarDriver->Start();
  }

  enable_get_calibration_thr_ = true;
  get_calibration_thr_ = new boost::thread(
      boost::bind(&LidarDriverSDK::GetCalibrationFromDevice, this));
      //相当于boost::function0< void> f =  boost::bind(&HelloWorld::hello,this);
          //或boost::function<void()> f = boost::bind(&HelloWorld::hello,this);
      //boost::thread thrd( f );
    
    //
}

void LidarDriverSDK::Stop() {
  if (lidarDriver) lidarDriver->Stop();

  enable_get_calibration_thr_ = false;
  if (get_calibration_thr_) {
    get_calibration_thr_->join();//使用Boost，如果我想要一个线程开始运行，我必须调用join()方法才能使线程运行。
  }
}

void LidarDriverSDK::GetCalibrationFromDevice() {
  // LOG_FUNC();
  if (!tcp_command_client_) {
    return;
  }
  std::cout << "Load correction file from lidar" << std::endl;
  int32_t ret = 0;
  // get lidar calibration.
  char *buffer = NULL;
  uint32_t len = 0;

  ret = TcpCommandGetLidarCalibration(tcp_command_client_, &buffer, &len);
  if (ret == 0 && buffer) {
    // success;
    correction_content_ = std::string(buffer);
    if (lidarDriver) {
      ret = lidarDriver->LoadCorrectionFile(correction_content_);
      if (ret != 0) {
        std::cout << "Load correction file from lidar failed" << std::endl;
      } else {
        std::cout << "Load correction file from lidar succeed" << std::endl;
        lidarDriver->SetCorrectionFileFlag(true);
      }
    }
    free(buffer);
  }
  if(!lidarDriver->GetCorrectionFileFlag()){
    std::ifstream fin(correction_file_path_);
    if (fin.is_open()) {
      std::cout << "Open correction file " << correction_file_path_ << " succeed" << std::endl;
    }
    else{
      std::cout << "Open correction file " << correction_file_path_ <<" failed" << std::endl;
      return;
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
    ret = lidarDriver->LoadCorrectionFile(strlidarCalibration);
    if (ret != 0) {
      std::cout << "Load correction file from " << correction_file_path_ <<" failed" << std::endl;
    } else {
      std::cout << "Load correction file from " << correction_file_path_ << " succeed" << std::endl;
      lidarDriver->SetCorrectionFileFlag(true);
    }
  }
}

void LidarDriverSDK::PushScanPacket(asensing_lidar::LidarScanPtr scan) {
  if (lidarDriver) {
    lidarDriver->PushScanPacket(scan);
  }
}
