#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <sys/time.h>
#include <sys/timeb.h>

class TranceFunc
{
public:
    TranceFunc(const char* file, const char* func){
        m_cFile = file;
        m_cFunc = func;
        struct tm *ptm;
        struct timeb stTimeb;
        ftime(&stTimeb);
        ptm = localtime(&stTimeb.time);
        printf("[T] %02d:%02d:%02d.%03d pid:%d tid:%10d ->[File:%s Function:%s ]\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm, getpid(), std::hash<std::thread::id>()(std::this_thread::get_id()), m_cFile, m_cFunc);
    }
    ~TranceFunc(){
        struct tm *ptm;
        struct timeb stTimeb;
        ftime(&stTimeb);
        ptm = localtime(&stTimeb.time);
        printf("[T] %02d:%02d:%02d.%03d pid:%d tid:%10d <-[File:%s Function:%s ]\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm, getpid(), std::hash<std::thread::id>()(std::this_thread::get_id()), m_cFile, m_cFunc);
    }
    const char* m_cFile;
    const char* m_cFunc;
};

#define LOG_D(format,...) {\
    struct tm *ptm;\
    struct timeb stTimeb;\
    ftime(&stTimeb);\
    ptm = localtime(&stTimeb.time);\
    printf("[D] %02d:%02d:%02d.%03d pid:%d tid:%10d File:%s Function:%s Line:%d " format"\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm, getpid(), std::hash<std::thread::id>()(std::this_thread::get_id()), __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);\
}

#define LOG_E(format,...) {\
    struct tm *ptm;\
    struct timeb stTimeb;\
    ftime(&stTimeb);\
    ptm = localtime(&stTimeb.time);\
    printf("[E] %02d:%02d:%02d.%03d pid:%d tid:%10d File:%s Function:%s Line:%d " format"\n", ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm, getpid(), std::hash<std::thread::id>()(std::this_thread::get_id()), __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);\
}

#define LOG_FUNC() TranceFunc tf( __FILE__, __FUNCTION__)
    