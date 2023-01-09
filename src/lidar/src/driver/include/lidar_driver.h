#ifndef INCLUDE_LIDAR_DRIVER_H_
#define INCLUDE_LIDAR_DRIVER_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <string>

#include <boost/function.hpp>

#include "point_types.h"
#include "asensing_lidar/LidarScan.h"

class LidarDriver_Internal;

class LidarDriver {
 public:
  /**
   * @brief Constructor
   * @param device_ip         The ip of the device
   * @param lidar_port        The port number of lidar data
   * @param pcl_callback      The callback of PCL data structure
   * @param start_angle       The start angle of every point cloud ,
   *                          should be <real angle> * 100.
   */
  LidarDriver(std::string device_ip, uint16_t lidar_port,
            boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)> pcl_callback,
            uint16_t start_angle,
            int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType, // the default timestamp type is LiDAR time
            std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag,
            std::string target_frame, std::string fixed_frame); 

  /**
   * @brief Constructor
   * @param pcap_path         The path of pcap file
   *        pcl_callback      The callback of PCL data structure
   *        start_angle       The start angle of every point cloud,
   *                          should be <real angle> * 100.
   *        tz                The timezone
   *        frame_id          The frame id of pcd
   */
  LidarDriver(std::string pcap_path, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)> pcl_callback, \
      uint16_t start_angle, int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,  // the default timestamp type is LiDAR time
      std::string lidar_correction_file, bool coordinate_correction_flag,
      std::string target_frame, std::string fixed_frame); 

  /**
   * @brief deconstructor
   */
  ~LidarDriver();

  /**
   * @brief load the lidar correction file
   * @param contents The correction contents of lidar correction
   */
  int LoadCorrectionFile(std::string contents);

  /**
   * @brief Reset Lidar's start angle.
   * @param angle The start angle
   */
  void ResetStartAngle(uint16_t start_angle);

  /**
   * @brief Run SDK.
   */
  void Start();

  /**
   * @brief Stop SDK.
   */
  void Stop();

  void PushScanPacket(asensing_lidar::LidarScanPtr scan);//推送扫描数据包

  bool GetCorrectionFileFlag();
  void SetCorrectionFileFlag(bool flag);

 private:
  LidarDriver_Internal *internal_;
};

#endif  /* INCLUDE_LIDAR_DRIVER_H_ */
