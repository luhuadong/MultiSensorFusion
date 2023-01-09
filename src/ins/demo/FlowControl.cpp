#include <chrono>
#include <thread>

#include "SerialPort.hpp"

using namespace std::chrono_literals;
using namespace CppLinuxSerial;

int main() {
    // This example relies on a serial device which echos serial data at 9600 baud, 8n1.
    std::cout << "FlowControll.cpp::main() called." << std::endl;
    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_230400, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE, HardwareFlowControl::OFF, SoftwareFlowControl::OFF);
    // serialPort.SetTimeout(1000); // Block when reading for 1000ms

	serialPort.Open();

    std::this_thread::sleep_for(100ms);

    std::thread t1([&]() {

        for (int x = 0; x < 10; x++) {
            // std::this_thread::sleep_for(100ms);
            std::cout << "Reading" << std::endl;
            std::string readData;
            serialPort.Read(readData);
            std::cout << "readData Length: " << readData.length() << std::endl;
        }
    });

    std::thread t2([&]() {

        std::this_thread::sleep_for(100ms);
        for (int x = 0; x < 10; x++) {
            std::this_thread::sleep_for(100ms);
            std::cout << "Writing \"Hello\"" << std::endl;
            serialPort.Write("Hello");
        }
    });

    t1.join();
    t2.join();
    
    serialPort.Close();
    return 0;
}