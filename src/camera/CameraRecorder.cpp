#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <gpiod.h>
#include "MvCameraControl.h"


#define GPIO1_B3          43 /* GPIO1_B3 = 1*32+8+3 = 43 */
#define RK_PB3            11

#define NO_TRIGGER        0
#define SOFTWARE_TRIGGER  1
#define HARDWARE_TRIGGER  2
#define TRIGGER_MODE      SOFTWARE_TRIGGER

#define UNUSED(x) (void)(x)

#define msleep(t) usleep((t)*1000)

static void *handle = NULL;
static struct gpiod_line *g_TriggerPin;
static bool g_bExit = false;

struct t_eventData {
    int myData;
};

pid_t gettid(void);
static void GpioTrigger(void);

static void TimerHandler(int sig, siginfo_t *si, void *uc)
{
    UNUSED(sig);
    UNUSED(uc);
    struct t_eventData *data = (struct t_eventData *) si->_sifields._rt.si_sigval.sival_ptr;
    printf("[%lu] Timer fired %d - thread-id: %d\n", time(NULL), ++data->myData, gettid());

#if (TRIGGER_MODE == SOFTWARE_TRIGGER)
    int nRet = MV_OK;

    nRet = MV_CC_SetCommandValue(handle, "TriggerSoftware");
    if(MV_OK != nRet) {
        printf("failed in TriggerSoftware[%x]\n", nRet);
    }
#elif (TRIGGER_MODE == HARDWARE_TRIGGER)
    GpioTrigger();
#endif
}

static int TimerInit(void)
{
    int res = 0;
    timer_t timerId = 0;

    struct sigevent sev = { 0 };
    struct t_eventData eventData = { .myData = 0 };

    /* specifies the action when receiving a signal */
    struct sigaction sa = { 0 };

    /* specify start delay and interval */
#if 0
    struct itimerspec its = {
        .it_value.tv_sec  = 1,
        .it_value.tv_nsec = 0,
        .it_interval.tv_sec  = 1,
        .it_interval.tv_nsec = 0
    };
#else
    struct itimerspec its = { {1, 0}, {1, 0} };
#endif

    printf("Signal Interrupt Timer - thread-id: %d\n", gettid());

    sev.sigev_notify = SIGEV_SIGNAL; // Linux-specific
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &eventData;

    /* create timer */
    res = timer_create(CLOCK_REALTIME, &sev, &timerId);

    if ( res != 0){
        fprintf(stderr, "Error timer_create: %s\n", strerror(errno));
        exit(-1);
    }

    /* specifz signal and handler */
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = TimerHandler;

    /* Initialize signal */
    sigemptyset(&sa.sa_mask);

    printf("Establishing handler for signal %d\n", SIGRTMIN);

    /* Register signal handler */
    if (sigaction(SIGRTMIN, &sa, NULL) == -1){
        fprintf(stderr, "Error sigaction: %s\n", strerror(errno));
        exit(-1);
    }

    /* start timer */
    res = timer_settime(timerId, 0, &its, NULL);

    if ( res != 0){
        fprintf(stderr, "Error timer_settime: %s\n", strerror(errno));
        exit(-1);
    }

    printf("Press ENTER to Exit\n");
    while(getchar()!='\n'){}
    return 0;
}

static int GpioInit(void)
{
    struct gpiod_chip *gpiochip1;
    //struct gpiod_line *pin;
    struct gpiod_line_request_config config;
    int req;

    /* 打开 GPIO 控制器 */
    gpiochip1 = gpiod_chip_open("/dev/gpiochip1");
    if (!gpiochip1) {
        fprintf(stderr, "GPIO chip open error.\n");
        return -1;
    }

    /* 获取 GPIO1_B3 引脚 */
    g_TriggerPin = gpiod_chip_get_line(gpiochip1, RK_PB3);

    if (!g_TriggerPin) {
        fprintf(stderr, "GPIO get line error.\n");
        gpiod_chip_close(gpiochip1);
        return -1;
    }

    /* 配置引脚，输出模式 name 为 "trigger" 初始电平为 low */
    req = gpiod_line_request_output(g_TriggerPin, "trigger", 0);
    if (req) {
        fprintf(stderr, "GPIO request error.\n");
        return -1;
    }

    return 0;
}

static void GpioTrigger(void)
{
    /* 上升沿触发 */
    printf("set pin trigger pulse\n");
    gpiod_line_set_value(g_TriggerPin, 1);
    msleep(100);
    gpiod_line_set_value(g_TriggerPin, 0);
}

// 等待用户输入enter键来结束取流或结束程序
// wait for user to input enter to stop grabbing or end the sample program
void PressEnterToExit(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
    fprintf(stderr, "\nPress enter to exit.\n");
    while (getchar() != '\n');
    g_bExit = true;
    sleep(1);
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}

static void *SaveThread(void *pUser)
{
    int nRet = MV_OK;
    unsigned char *pDataForSaveImage = NULL;
    int num_picture = 0;

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return NULL;
    }

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData)
    {
        return NULL;
    }
    unsigned int nDataSize = stParam.nCurValue;

    while (1)
    {
        /*nRet = MV_CC_SetCommandValue(pUser, "TriggerSoftware");
        if(MV_OK != nRet)
        {
            printf("failed in TriggerSoftware[%x]\n", nRet);
        }*/
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if (NULL == pData)
        {
            break;
        }
        unsigned int nDataSize = stParam.nCurValue;

        nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 5000);
        if (nRet == MV_OK)
        {
            printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
                   stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);

            pDataForSaveImage = (unsigned char *)malloc(stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048);
            if (NULL == pDataForSaveImage)
            {
                break;
            }
            // 填充存图参数
            // fill in the parameters of save image
            MV_SAVE_IMAGE_PARAM_EX stSaveParam;
            memset(&stSaveParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
            // 从上到下依次是：输出图片格式，输入数据的像素格式，提供的输出缓冲区大小，图像宽，
            // 图像高，输入数据缓存，输出图片缓存，JPG编码质量
            // Top to bottom are：
            stSaveParam.enImageType = MV_Image_Jpeg;
            stSaveParam.enPixelType = stImageInfo.enPixelType;
            stSaveParam.nBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 4 + 2048;
            stSaveParam.nWidth = stImageInfo.nWidth;
            stSaveParam.nHeight = stImageInfo.nHeight;
            stSaveParam.pData = pData;
            stSaveParam.nDataLen = stImageInfo.nFrameLen;
            stSaveParam.pImageBuffer = pDataForSaveImage;
            stSaveParam.nJpgQuality = 80;
            nRet = MV_CC_SaveImageEx2(pUser, &stSaveParam);
            if (MV_OK != nRet)
            {
                printf("failed in MV_CC_SaveImage,nRet[%x]\n", nRet);
                break;
            }

            num_picture++;
            char pic_name[20];
            int ret = snprintf(pic_name, 20, "%s%d%s", "image", num_picture, ".jpeg");
            FILE *fp = fopen(pic_name, "wb");
            if (NULL == fp)
            {
                printf("fopen failed\n");
                break;
            }
            fwrite(pDataForSaveImage, 1, stSaveParam.nImageLen, fp);
            fclose(fp);
            printf("save image succeed\n");
        }
        else
        {
            printf("Get One Frame failed![%x]\n", nRet);
        }
        if (g_bExit)
        {
            break;
        }
    }

    if (pData)
    {
        free(pData);
        pData = NULL;
    }

    return 0;
}

int main(int argc, char const *argv[])
{
    int nRet = MV_OK;

    do
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            break;
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            break;
        }

        printf("Please Intput camera index: ");
        unsigned int nIndex = 0;
        scanf("%d", &nIndex);

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Intput error!\n");
            break;
        }

        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            break;
        }

        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            break;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }

        /*nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
        if (MV_OK != nRet)
        {
            printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
            break;
        }*/

#if (TRIGGER_MODE != NO_TRIGGER)
        // 设置触发模式为on
        // set trigger mode as on
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            break;
        }

#if (TRIGGER_MODE == HARDWARE_TRIGGER)
        // 设置触发源（默认上升沿触发）
        // set trigger source as Line 2 (default Rising Edge)
        nRet = MV_CC_SetEnumValue(handle, "TriggerSource", 2);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
            break;
        }
#endif

#if (TRIGGER_MODE == SOFTWARE_TRIGGER)
        nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
            break;
        }
#endif
#endif /* NO_TRIGGER */

        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            break;
        }

        pthread_t saveThreadID;
        nRet = pthread_create(&saveThreadID, NULL, SaveThread, handle);
        if (nRet != 0)
        {
            printf("create save thread failed.ret = %d\n", nRet);
            break;
        }

#if (TRIGGER_MODE != NO_TRIGGER)
#if (TRIGGER_MODE == HARDWARE_TRIGGER)
        if (0 != GpioInit()) {
            printf("GPIO init failed. Exit\n");
            return -1;
        }
#endif
        TimerInit();
#endif
        PressEnterToExit();

        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            break;
        }

        // 关闭设备
        // close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            break;
        }

        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            break;
        }
    } while (0);

    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }

    printf("exit.\n");
    return 0;
}
