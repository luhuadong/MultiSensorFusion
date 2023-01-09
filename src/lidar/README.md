# ASENSING_LiDAR_Driver

Asensing Lidar General SDK for A0 LiDAR sensor.

## Build

```bash
cd <project>
mkdir build
cd build
cmake ..
make
```
## Add to your project
### Cmake

```cmake
add_subdirectory(<path_to>ASENSING_LiDAR_Driver)

include_directories(
	<path_to>ASENSING_LiDAR_Driver/api
	<path_to>ASENSING_LiDAR_Driver/src/driver/include
)

target_link_libraries(<Your project>
  LidarDriverSDK
)
```
### C++

```cpp
#include "lidar_driver_sdk.h"

// for A0
LidarDriverSDK lidarDriver(std::string("192.168.101.101"), 51180, \
    lidarCallback, 0, 0, 1, std::string("A0"));
```
