#ifndef SRC_INPUT_H_
#define SRC_INPUT_H_

#include <netinet/in.h>
#include <stdio.h>
#include <unistd.h>
#include <string>

#define ETHERNET_MTU (5000)

typedef struct LidarPacket_s {
  double stamp;
  uint8_t data[ETHERNET_MTU];
  uint32_t size;
} LidarPacket;

class Input {
 public:
  Input(uint16_t port, std::string multicast_ip = "");
  ~Input();
  Input(std::string filePath, int type);
  int getPacket(LidarPacket *pkt);

 private:
  int socketForLidar;//雷达的socket
  int socketNumber;
};

#endif  // SRC_INPUT_H_
