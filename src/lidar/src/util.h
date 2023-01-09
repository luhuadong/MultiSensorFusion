#ifndef SRC_UTIL_H_
#define SRC_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

int sys_readn(int fd, void* vptr, int n);
int sys_writen(int fd, const void* vptr, int n);
int tcp_open(const char* ipaddr, int port);
int select_fd(int fd, int timeout, int wait_for);
double getNowTimeSec();

enum { WAIT_FOR_READ, WAIT_FOR_WRITE, WAIT_FOR_CONN };

#ifdef __cplusplus
}
#endif

#endif  /* SRC_UTIL_H_ */
