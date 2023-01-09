#include "lidar_driver_sdk.h"

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp) {
  printf("Frame timestamp: %lf,\n", timestamp);
  printf("point_size: %ld,\n",cld->points.size());
}

int main(int argc, char** argv)
{
  // LidarDriverSDK lidarDriver(std::string("192.168.1.201"), 51180, lidarCallback, 0, 0, 1, std::string("A0"));
  LidarDriverSDK lidarDriver(std::string("/path/to/pcapFile"), lidarCallback, 0, 0, 1, std::string("A0"));

  std::string filePath = "/path/to/correctionFile";
  std::ifstream fin(filePath);
  int length = 0;
  std::string strlidarCalibration;
  fin.seekg(0, std::ios::end);
  length = fin.tellg();
  fin.seekg(0, std::ios::beg);
  char *buffer = new char[length];
  fin.read(buffer, length);
  fin.close();
  strlidarCalibration = buffer;
  lidarDriver.LoadLidarCorrectionFile(strlidarCalibration);

  lidarDriver.Start();

  while (true) {
    sleep(100);
  }

  return 0;
}
