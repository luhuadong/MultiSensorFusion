#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <iostream>
#include <sstream>
#include <time.h>
#include "../util.h"

#include "input.h"
#include "log.h"

Input::Input(uint16_t port, std::string multicast_ip)
{
  socketForLidar = -1;
  socketForLidar = socket(PF_INET, SOCK_DGRAM, 0);
  if (socketForLidar == -1) {
    perror("socket");  // TODO(Philip.Pi): perror errno.
    return;
  }

  sockaddr_in myAddress;                     // my address information
  memset(&myAddress, 0, sizeof(myAddress));  // initialize to zeros
  myAddress.sin_family = AF_INET;            // host byte order
  myAddress.sin_port = htons(port);          // port in network byte order
  myAddress.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

  if (bind(socketForLidar, reinterpret_cast<sockaddr *>(&myAddress),
           sizeof(sockaddr)) == -1) {
    perror("bind");  // TODO(Philip.Pi): perror errno
    return;
  }
  if(multicast_ip != ""){
    struct ip_mreq mreq;                      // 多播地址结构体
    mreq.imr_multiaddr.s_addr=inet_addr(multicast_ip.c_str());
    mreq.imr_interface.s_addr = htonl(INADDR_ANY); 
    int ret = setsockopt(socketForLidar, IPPROTO_IP, IP_ADD_MEMBERSHIP, (const char *)&mreq, sizeof(mreq));
    if (ret < 0) {
      perror("Multicast IP error,set correct multicast ip address or keep it empty in lanch file\n");
    } 
    else {
      printf("Recive data from multicast ip address %s\n", multicast_ip.c_str());
    }
  }

  if (fcntl(socketForLidar, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    perror("non-block");
    return;
  }

  socketNumber = 1;
}

Input::~Input(void)
{
  if (socketForLidar > 0) {
    (void)close(socketForLidar);
  }
}

// return : 0 - lidar
//          -1 - error
int Input::getPacket(LidarPacket *pkt)
{
  struct pollfd fds[socketNumber];

  /* socketNumber == 1 */
  fds[0].fd = socketForLidar;
  fds[0].events = POLLIN;

  static const int POLL_TIMEOUT = 1000;  // one second (in msec)

  sockaddr_in senderAddress;
  socklen_t senderAddressLen = sizeof(senderAddress);
  int retval = poll(fds, socketNumber, POLL_TIMEOUT);
  if (retval < 0) {  // poll() error?
    if (errno != EINTR) printf("poll() error: %s", strerror(errno));
    return -1;
  }
  if (retval == 0) {
    return -1;
  }
  if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) ||
      (fds[0].revents & POLLNVAL)) {
    // device error?
    perror("poll() reports packet error");
    return -1;
  }

  senderAddressLen = sizeof(senderAddress);
  ssize_t nbytes;
  double time = getNowTimeSec();
  // printf("Real time: %lf\n",time);
  for (int i = 0; i != socketNumber; ++i) {
    if (fds[i].revents & POLLIN) {
      nbytes = recvfrom(fds[i].fd, &pkt->data[0], ETHERNET_MTU, 0,
                        reinterpret_cast<sockaddr *>(&senderAddress),
                        &senderAddressLen);
      break;
    }
  }

  if (nbytes < 0) {
    if (errno != EWOULDBLOCK) {
      perror("recvfail");
      return -1;
    }
  }
  pkt->size = nbytes;
  pkt->stamp = time;

  return 0;
}
