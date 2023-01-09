#ifndef VERSION_H_
#define VERSION_H_

#include <stdio.h>
#include <unistd.h>
#include <string>

#define VERSION "1.0.0"
#ifdef __cplusplus
extern "C" {
#endif

void printVersion() {
    printf("       ///////////////////////////////////////\n"
           "       //     LiDAR SDK version: %s      //\n" 
           "       ///////////////////////////////////////\n",VERSION);
}

#ifdef __cplusplus
}
#endif

#endif  // VERSION_H_

