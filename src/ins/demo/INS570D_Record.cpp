#include <iostream>
#include "INS570Driver.hpp"

int main(int argc, char *argv[])
{
    std::string SerialName;
    std::vector<uint8_t> data;
    bool ret;

    std::cout << "== INS570 Record Tool ==" << std::endl;

    if (argc > 1) {
        SerialName = argv[1];
    }
    else {
        SerialName = "/dev/ttyUSB0";
    }

    INS570_Driver ins570d(SerialName);

    ret = ins570d.Open();

    if (!ret) {
        AG_ERROR << "Serial port open failed" << AG_REND;
        return -1;
    }

    AG_INFO << "Recording..." << AG_REND;
    ins570d.Start("INS570D.csv", INS570_Format::CSV);
    std::this_thread::sleep_for(std::chrono::seconds(10));
    ins570d.Stop();

    return 0;
}