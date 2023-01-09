#ifndef INS570_DRIVER_H_
#define INS570_DRIVER_H_

#include <iostream>
#include <string>
#include <deque>
#include <thread>
#include <cstdint>

#include "SerialPort.hpp"
#include "AGLog.hpp"

using namespace CppLinuxSerial;

// #pragma pack(push, 1)
typedef struct
{
    uint64_t sys_timestamp;
    uint64_t gps_timestamp;
    float roll;   /* 横滚角 */
    float pitch;  /* 俯仰角 */
    float yaw;    /* 方位角 */
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acce_x;
    float acce_y;
    float acce_z;
    double latitude;
    double longitude;
    double altitude;
    float vel_n;  /*  */
    float vel_e;  /*  */
    float vel_d;  /*  */
    uint8_t status;
    uint8_t type;
    int16_t data1;
    int16_t data2;
    int16_t data3;
    double gpst;
    uint32_t gpsw;

} InsData;
// #pragma pack(pop)

enum class INS570_State {
    FRAME_BEGIN,
    FRAME_START1,
    FRAME_START2,
    FRAME_START3,
    FRAME_BODY,
    FRAME_END,
};

enum class INS570_Format {
    RAW,
    CSV,
    JSON,
};

class INS570_Driver
{
public:
    INS570_Driver(const std::string &serialName);
    ~INS570_Driver();

    bool Open(void);
    bool Start(void);
    bool Start(std::string filename, INS570_Format format);
    bool Stop(void);

    int Recv(std::vector<uint8_t> &data, const size_t len);
    int Send(std::vector<uint8_t> &data, const size_t len);

private:
    void parseInsData(void);
    void printInsData(void);
    void saveInsData(std::string filename);

    CppLinuxSerial::SerialPort *_serialPort;
    std::deque<InsData> insData_deq;

    INS570_State _state;
    std::thread* parseThread;
    std::thread* printThread;
    std::thread* saveThread;
    bool flag = true;
};

#endif /*INS570_DRIVER_H_ */