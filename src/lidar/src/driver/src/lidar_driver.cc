#include "lidar_driver.h"
#include "lidar_driver_internal.h"
#include "log.h"
/**
 * @brief Constructor
 * @param device_ip         The ip of the device
 *        lidar_port        The port number of lidar data
 *        pcl_callback      The callback of PCL data structure
 *        start_angle       The start angle of every point cloud
 */
LidarDriver::LidarDriver(//连接雷达的
    std::string device_ip, uint16_t lidar_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)> pcl_callback,
    uint16_t start_angle, int tz,
    int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,
    std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag,
    std::string target_frame, std::string fixed_frame) {
      // LOG_FUNC();
  internal_ =
      new LidarDriver_Internal(device_ip, lidar_port, pcl_callback,
                             start_angle, tz, pcl_type, lidar_type, frame_id, timestampType, lidar_correction_file, 
                             multicast_ip, coordinate_correction_flag, target_frame, fixed_frame);
}

/**
 * @brief Constructor
 * @param pcap_path         The path of pcap file
 *        pcl_callback      The callback of PCL data structure
 *        start_angle       The start angle of every point cloud
 *        tz                The timezone
 *        frame_id          The frame id of point cloud
 */
LidarDriver::LidarDriver(
    std::string pcap_path, \
    boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)> pcl_callback,\
    uint16_t start_angle, int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType, 
    std::string lidar_correction_file, bool coordinate_correction_flag, std::string target_frame, std::string fixed_frame) {
  internal_ = new LidarDriver_Internal(pcap_path, pcl_callback, start_angle, \
      tz, pcl_type, lidar_type, frame_id, timestampType, lidar_correction_file, \
      coordinate_correction_flag, target_frame, fixed_frame);
}

/**
 * @brief deconstructor
 */
LidarDriver::~LidarDriver() { delete internal_; }

/**
 * @brief load the lidar correction file
 * @param contents The correction contents of lidar correction
 */
int LidarDriver::LoadCorrectionFile(std::string file) {
  return internal_->LoadCorrectionFile(file);
}

/**
 * @brief Reset Lidar's start angle.
 * @param angle The start angle
 */
void LidarDriver::ResetStartAngle(uint16_t start_angle) {
  internal_->ResetStartAngle(start_angle);
}

/**
 * @brief Run SDK.
 */
void LidarDriver::Start() { internal_->Start(); }

/**
 * @brief Stop SDK.
 */
void LidarDriver::Stop() { internal_->Stop(); }

void LidarDriver::PushScanPacket(asensing_lidar::LidarScanPtr scan) {
  if (internal_) {
    internal_->PushScanPacket(scan);
  }
}

bool LidarDriver::GetCorrectionFileFlag(){
  if (internal_) {
    return internal_->GetCorrectionFileFlag();
  }
}

void LidarDriver::SetCorrectionFileFlag(bool flag){
  if (internal_) {
    internal_->SetCorrectionFileFlag(flag);
  }
}
