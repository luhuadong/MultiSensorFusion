#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#include "INS570Driver.hpp"
#include "GPSTime.hpp"

constexpr char HEADER_0B[3] = {(char)0xBD, (char)0xDB, (char)0x0B};
constexpr int LENGTH_PACKET = 63;

INS570_Driver::INS570_Driver(const std::string &serialName)
{
    AG_DEBUG << "Serial Port: " << serialName << AG_REND;

    _serialPort = new SerialPort(serialName, BaudRate::B_230400, NumDataBits::EIGHT, Parity::NONE, 
                                 NumStopBits::ONE, HardwareFlowControl::OFF, SoftwareFlowControl::OFF);

}

INS570_Driver::~INS570_Driver()
{
    delete _serialPort;
}

bool INS570_Driver::Open(void)
{
    return _serialPort->Open();
}

bool INS570_Driver::Start(void)
{
    /* Print data */
    flag = true;

    AG_INFO << "INS570 driver start" << AG_REND;

    _state = INS570_State::FRAME_BEGIN;

    parseThread = new std::thread(&INS570_Driver::parseInsData, this);
    printThread = new std::thread(&INS570_Driver::printInsData, this);

    return true;
}

bool INS570_Driver::Start(std::string filename, INS570_Format format)
{
    /* Record data */
    flag = true;
    
    AG_INFO << "INS570 driver start record" << AG_REND;

    parseThread = new std::thread(&INS570_Driver::parseInsData, this);
    saveThread = new std::thread(&INS570_Driver::saveInsData, this, filename);

    return true;
}

bool INS570_Driver::Stop(void)
{
    AG_INFO << "INS570 Driver Stop" << AG_REND;

    flag = false;
    parseThread->join();
    saveThread->join();

    _serialPort->Close();

    return true;
}

int INS570_Driver::Recv(std::vector<uint8_t> &data, const size_t len)
{
    AG_INFO << "INS570 Driver Read" << AG_REND;

    return _serialPort->ReadBinary(data);
}

int INS570_Driver::Send(std::vector<uint8_t> &data, const size_t len)
{
    AG_INFO << "INS570 Driver Write" << AG_REND;

    return _serialPort->WriteBinary(data);
}

void INS570_Driver::saveInsData(std::string filename)
{
    pthread_setname_np(pthread_self(), "saveInsData");

    InsData data570d_t;
    std::ofstream outFile;
    outFile.open(filename, std::ios::out);
    outFile << "sys_timestamp" << "," << "roll" << "," << "pitch" << "," << "yaw" << ","
            << "gxf" << "," << "gyf" << "," << "gzf" << "," << "ax" << "," << "ay" << ","
            << "az" << "," << "lat" << "," << "lon" << "," << "alt" << "," << "Nvel" << ","
            << "Evel" << "," << "Dvel" << "," << "status" << "," << "data1" << "," << "data2"
            << "," << "data3" << "," << "gpst" << "," << "gpsweek" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    while (flag)
    {
        if (insData_deq.size() > 0)
        {
            data570d_t = insData_deq.front();
            outFile << data570d_t.sys_timestamp << "," << data570d_t.roll << "," << data570d_t.pitch << "," << data570d_t.yaw << ","
                    << data570d_t.gyro_x << "," << data570d_t.gyro_y << "," << data570d_t.gyro_z << "," << data570d_t.acce_x << "," << data570d_t.acce_y
                    << "," << data570d_t.acce_z << "," << data570d_t.latitude << "," << data570d_t.longitude << "," << data570d_t.altitude << ","
                    << data570d_t.vel_n << "," << data570d_t.vel_e << "," << data570d_t.vel_d << "," << data570d_t.status << "," << data570d_t.data1
                    << "," << data570d_t.data2 << "," << data570d_t.data3 << "," << data570d_t.gpst << "," << data570d_t.gpsw << std::endl;
            insData_deq.pop_front();
        }
        usleep(5000);
    }

    outFile.close();
}

void INS570_Driver::printInsData(void)
{
    pthread_setname_np(pthread_self(), "printInsData");

    InsData ins;

    while (flag)
    {
        if (insData_deq.size() > 0)
        {
            ins = insData_deq.front();

            std::cout << "\nINS Data >> " << std::endl;
            std::cout << "sys_timestamp: " << ins.sys_timestamp << "\ngps_timestamp: " << ins.gps_timestamp 
                      << "\nroll: " << ins.roll << ", pitch: " << ins.pitch << ", yaw: " << ins.yaw 
                      << "\ngx: " << ins.gyro_x << ", gy: " << ins.gyro_y << ", gz: " << ins.gyro_z 
                      << "\nax: " << ins.acce_x << ", ay: " << ins.acce_y << ", az: " << ins.acce_z 
                      << "\nlat: " << ins.latitude << ", lon: " << ins.longitude << ", alt: "  << ins.altitude 
                      << "\nvn: " << ins.vel_n << ", ve: " << ins.vel_e << ", vd: " << ins.vel_d 
                      << "\ndata1: " << ins.data1 << ", data2: " << ins.data2 << ", data3: " << ins.data3 
                      << "\ngpst: " << ins.gpst << ", gpsw: " << ins.gpsw  << "\nstatus: " << ins.status << std::endl;
            insData_deq.pop_front();
        }
        usleep(5000);
    }
}

void INS570_Driver::parseInsData(void)
{
    pthread_setname_np(pthread_self(), "parseInsData");

    int read_in_num = 0;
    char uart_buf[256]{0};
    // char send_str[LENGTH_PACKET]{0};
    InsData data_570d;

    timeval  cur_time;
    uint64_t sys_timestamp;
    uint64_t gps_timestamp;
    uint64_t cur_seconds;      /* seconds */
    uint64_t cur_microseconds; /* microseconds  微秒*/

    PCOMMONTIME pct = new COMMONTIME;
    PGPSTIME pgt = new GPSTIME;

    int16_t roll, pitch, yaw, gx, gy, gz, ax, ay, az, Nvel, Evel, Dvel;
    int16_t ins_status, data1, data2, data3, pdata_type;
    int32_t latitude, longitude, altitude;
    uint32_t gpst, gpsw;

    float rollf, pitchf, yawf, gxf, gyf, gzf, axf, ayf, azf, Nvelf, Evelf, Dvelf;
    double latitudef, longitudef, altitudef, gpstf, gpswf;

    std::string input;
    std::string data;

    while (flag)
    {
        AG_DEBUG << "Parse INS Data" << AG_REND;

        read_in_num = _serialPort->Read(data);
        input.append(data.c_str(), read_in_num);

        if (read_in_num > 0)
            std::cout << "read " << read_in_num << " bytes." << std::endl;
        if (read_in_num == 255)
        {
            std::cout << "you should poll uart faster!" << std::endl;
        }
        while (input.size() >= 3)
        {
            auto header_pos = input.find(HEADER_0B, 0, 3);
            if (header_pos == std::string::npos)
            {
                break;
            }
            else if (header_pos == 0) // find DBBD0B
            {
                if (input.size() < LENGTH_PACKET)
                    break;
                int xorcheck = 0;
                for (int i = 0; i < LENGTH_PACKET - 1; i++)
                {
                    xorcheck = xorcheck ^ input[i];
                }
                if (input[LENGTH_PACKET - 1] == xorcheck)
                {
                    gettimeofday(&cur_time, NULL);
                    sys_timestamp = cur_time.tv_sec * 1000 + cur_time.tv_usec / 1000;

                    // get RPY
                    roll  = ((0xff & (char)input[4]) << 8) | (0xff & (char)input[3]);
                    pitch = ((0xff & (char)input[6]) << 8) | (0xff & (char)input[5]);
                    yaw   = ((0xff & (char)input[8]) << 8) | (0xff & (char)input[7]);

                    // calculate RPY in deg
                    rollf  = roll  * (360.0 / 32768); // 32768=2^15, 1000 0000 0000 0000,???
                    pitchf = pitch * (360.0 / 32768);
                    yawf   = yaw   * (360.0 / 32768);

                    // get gyro values
                    gx = ((0xff & (char)input[10]) << 8) | (0xff & (char)input[9]);
                    gy = ((0xff & (char)input[12]) << 8) | (0xff & (char)input[11]);
                    gz = ((0xff & (char)input[14]) << 8) | (0xff & (char)input[13]);

                    // calculate gyro in deg/s
                    gxf = gx * 300.0 / 32768;
                    gyf = gy * 300.0 / 32768;
                    gzf = gz * 300.0 / 32768;

                    // get acelerometer values
                    ax = ((0xff & (char)input[16]) << 8) | (0xff & (char)input[15]);
                    ay = ((0xff & (char)input[18]) << 8) | (0xff & (char)input[17]);
                    az = ((0xff & (char)input[20]) << 8) | (0xff & (char)input[19]);

                    // calculate acelerometer in g
                    axf = ax * 12.0 / 32768;
                    ayf = ay * 12.0 / 32768;
                    azf = az * 12.0 / 32768;

                    // get gps values
                    latitude = (((0xff & (char)input[24]) << 24) | ((0xff & (char)input[23]) << 16) |
                                ((0xff & (char)input[22]) << 8) | 0xff & (char)input[21]);
                    longitude = (((0xff & (char)input[28]) << 24) | ((0xff & (char)input[27]) << 16) |
                                 ((0xff & (char)input[26]) << 8) | 0xff & (char)input[25]);
                    altitude = (((0xff & (char)input[32]) << 24) | ((0xff & (char)input[31]) << 16) |
                                ((0xff & (char)input[30]) << 8) | 0xff & (char)input[29]);

                    // calculate lat、lon in deg(WGS84)
                    latitudef  = latitude * 1e-7;
                    longitudef = longitude * 1e-7;
                    // calculate lat、lon in m
                    altitudef  = altitude * 1e-3;

                    // get  NED vel values
                    Nvel = ((0xff & (char)input[34]) << 8) | (0xff & (char)input[33]);
                    Evel = ((0xff & (char)input[36]) << 8) | (0xff & (char)input[35]);
                    Dvel = ((0xff & (char)input[38]) << 8) | (0xff & (char)input[37]);

                    // calculate NED vel in m/s
                    Nvelf = Nvel * (100.0 / 32768);
                    Evelf = Evel * (100.0 / 32768);
                    Dvelf = Dvel * (100.0 / 32768);

                    // ins_status values
                    ins_status = (0x0f & (char)input[39]);

                    // data 1-3
                    data1 = ((0xff & (char)input[47]) << 8) | (0xff & (char)input[46]);
                    data2 = ((0xff & (char)input[49]) << 8) | (0xff & (char)input[48]);
                    data3 = ((0xff & (char)input[51]) << 8) | (0xff & (char)input[50]);

                    // get gps time values
                    gpst = (((0xff & (char)input[55]) << 24) | ((0xff & (char)input[54]) << 16) |
                            ((0xff & (char)input[53]) << 8) | 0xff & (char)input[52]);
                    // calculate gps time in ms
                    gpstf = gpst * 0.25;

                    // pdata_type
                    pdata_type = (0xff & (char)input[56]);

                    // get gps week values
                    gpsw = (((0xff & (char)input[61]) << 24) | ((0xff & (char)input[60]) << 16) |
                            ((0xff & (char)input[59]) << 8) | 0xff & (char)input[58]);
                    
                    gpswf = gpsw * 1.0;

                    pgt->weeks = gpsw;
                    pgt->tow.seconds = floor(gpstf/1000);
                    pgt->tow.tos = gpstf/1000 - pgt->tow.seconds;
                    GPSTimeToCommonTime(pgt, pct);

                    AG_INFOL << "GPS Time: " << pct->year << "-" << pct->month << "-" << pct->day
                             << " " << pct->hour << ":" << pct->minute << ":" << pct->second << AG_REND;

                    data_570d.sys_timestamp = sys_timestamp;
                    data_570d.gps_timestamp = (gpswf * 7 * 24 * 60 * 60 * 1000) + gpstf;
                    data_570d.roll = rollf;
                    data_570d.pitch = pitchf;
                    data_570d.yaw = yawf;
                    data_570d.gyro_x = gxf;
                    data_570d.gyro_y = gyf;
                    data_570d.gyro_z = gzf;
                    data_570d.acce_x = axf;
                    data_570d.acce_y = ayf;
                    data_570d.acce_z = azf;
                    data_570d.longitude = longitudef;
                    data_570d.latitude = latitudef;
                    data_570d.altitude = altitudef;
                    data_570d.vel_n = Nvelf;
                    data_570d.vel_e = Evelf;
                    data_570d.vel_d = Dvelf;
                    data_570d.status = ins_status;
                    data_570d.data1 = data1;
                    data_570d.data2 = data2;
                    data_570d.data3 = data3;
                    data_570d.gpst = gpstf;
                    data_570d.gpsw = gpsw;

                    insData_deq.push_back(data_570d);
                }
                input.erase(0, LENGTH_PACKET);
            }
            else
            {
                size_t frame_0B_pos = input.find(HEADER_0B, 0, 3);
                if (frame_0B_pos == std::string::npos)
                {
                    input.erase(0, input.size());
                }
                else
                {
                    // printf("find frame\n");
                    input.erase(0, frame_0B_pos);
                }
            }
        }
    } /* End of while */

    delete pgt;
    delete pct;
}
