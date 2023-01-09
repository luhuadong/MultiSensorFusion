#include <iostream>
#include "INS570Driver.hpp"

int main(int argc, char *argv[])
{
    std::string SerialName;
    std::vector<uint8_t> data;
    bool ret;

    std::cout << "== INS570 Dump Tool ==" << std::endl;

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

    ins570d.Start();

    while (1)
    {
        ins570d.Recv(data, 1);

        std::cout << "Data size: " << std::dec << data.size() << std::endl;

#if 1
        for (uint8_t n : data)
        {
            // std::cout << "0x" << std::hex << static_cast<uint16_t>(n) << " ";
            printf("%02X ", n);
        }
        printf("\n");
#endif
        std::cout << std::endl;
    }

    ins570d.Stop();

    return 0;
}