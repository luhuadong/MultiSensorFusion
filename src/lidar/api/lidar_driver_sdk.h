#ifndef LIDAR_DRIVER_SDK_H_
#define LIDAR_DRIVER_SDK_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <string>
#include <vector>

#include <boost/function.hpp>

#include "lidar_driver.h"
#include "point_types.h"
#include "asensing_lidar/LidarScan.h"
#include <boost/thread.hpp>

class LidarDriverSDK {
 public:
  /**
   * @brief Constructor
   * @param device_ip         The ip of the device
   * @param lidar_port        The port number of lidar data
   * @param pcl_callback      The callback of PCL data structure
   * @param start_angle       The start angle of every point cloud
   *                          should be <real angle> * 100.
   * 
   * boost::function是一个函数对象的“容器”，概念上像是C/C++中函数指针类型的泛化，是一种“智能函数指针”。
   *    它以对象的形式封装了原始的函数指针或函数对象，能够容纳任意符合函数签名的可调用对象。
   *    因此，它可以被用于回调机制，暂时保管函数或函数对象，在之后需要的时机在调用，使回调机制拥有更多的弹性。
   *    boost::function<float (int x, int y)> func1;
   *        即，
   *        该模板的函数，
   *        ，
   *            使用起来就是
   *            #include<boost/function.hpp>
   *            #include<iostream>
   *            using namespace std;
   * 
   *            typedef boost::function<int(int ,char)> Func;//该模板为Func,类型float,该模板的函数的参数 (int ,char )
   * 
   *            int test(int num,char sign)
   *            {
   *                cout << num << sign << endl;
   *                return 0;
   *            }
   *            int main()
   *            {
   *                Func f;
   *                f = &test;  //or f = test;绑定功能函数
   *                f(1, 'A');
   *            }
   * 
   *function可以配合bind使用，存储bind表达式的结果，使bind可以被多次调用
   *    如：class Foo{
   *            void methodA()....
   *            void methodInt(int a)........
   *            int methodTest(int a, char b, int c).......
   * 
   *    //无参数，无返回值
   *    boost::function<void()> fun1;
   *    //调用foo.methodA()
   *    Foo foo;
   *    fun1 = boost::bind(&Foo::methodA, &foo);
   *    fun1();
   * 
   *    //调用foo.methodInt(int a)
   *    Foo foo;
   *    fun1 = boost::bind(&Foo::methodInt, &foo ,40);
   *    fun1();
   * 
   *    ////有参数
   *    boost::function<int(int, int)> func3;
   *    //bind的时候未传入实参，需要_1、_2、_3作为参数的占位
   *    //下面传入一个实参，其他的用_n做占位符，很明显是一个闭包
   *    Foo foo;
   *    func3 = boost::bind(&Foo::methodTest, &foo, _1, 'z', _2);
   *    func3(1, 2);
   * 
   * bind接收的第一个参数必须是一个可调用的对象f，包括函数、函数指针、函数对象、和成员函数指针
   *    之后bind最多接受9个参数，参数数量必须与f的参数数量相等，这些参数被传递给f作为入参。
   *    绑定完成后，bind会返回一个函数对象，它内部保存了f的拷贝，具有operator()，返回值类型被自动推导为f的返回类型。
   *    在发生调用时这个函数对象将把之前存储的参数转发给f完成调用。
   *        如，有一个函数func，它的形式是：func(a1,a2);
   *            他将等价于一个具有无参operator()的bind函数对象调用  ：bind(func,a1,a2)();
   *        bind的真正威力在于它的占位符，它们分别定义为_1,_2,_3,一直到 _9,位于一个匿名的名字空间。
   *            占位符可以取代bind参数的位置，在发生调用时才接受真正的参数。
   *            占位符的名字表示它在调用式中的顺序，而在绑定的表达式中没有没有顺序的要求，_1不一定必须第一个出现，也不一定只出现一次
   *                如  bind(func,_2,_1)(a1,a2)
   *                    返回一个具有两个参数的函数对象，第一个参数将放在func的第二个位置，而第二个参数则放在第一个位置，调用时等价于：func(a2,a1);
   * 
   * boost::function + boost::bind 描述了一对一的回调，
   * 在项目中，我们借助boost::shared_ptr + boost::weak_ptr简洁地实现了多播(multi-cast)，即一对多的回调，
   * 
   * 
   * boost::shared_ptr是可以共享所有权的指针。如果有多个shared_ptr共同管理同一个对象时，只有这些shared_ptr全部与该对象脱离关系之后，被管理的对象才会被释放
   * 对所管理的对象进行了引用计数
   * 在这里boost::shared_ptr<PPointCloud>指向的就是&AgLidarClient::lidarCallback
   * 
   * boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)>pcl_callback
   *    模板pcl_callback，参数为boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr
   * 其在main.cc中的输入为boost::bind(&AgLidarClient::lidarCallback, this, _1, _2, _3)，
   *    即对象this绑定的&AgLidarClient::lidarCallback函数，参数为_1, _2, _3， 对应上述三个参数
   * 
   * 从而将&AgLidarClient::lidarCallback作为参数传给pcl_callback
   * 
   * boost::function<void(double)> gps_callback
   *    模板gps_callback，参数为double
   * 其在main.cc中的输入为boost::bind(&AgLidarClient::gpsCallback, this, _1)
   *    即对象this绑定的&AgLidarClient::gpsCallback,函数，参数为_1 对应上述参数 double
   *        
   */
  LidarDriverSDK(//连接雷达模式的
      std::string device_ip, const uint16_t lidar_port,
      boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)>
          pcl_callback,
      uint16_t start_angle,
      int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,  // the default timestamp type is LiDAR time
      std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag, 
      std::string target_frame, std::string fixed_frame);
  /**
   * @brief Constructor
   * @param pcap_path         The path of pcap file
   *        pcl_callback      The callback of PCL data structure
   *        start_angle       The start angle of every point cloud
   *                          should be <real angle> * 100.
   *        tz                The timezone
   *        frame_id          The frame id of point cloud
   */
  LidarDriverSDK(//连接pcap文件或rosbag包的
      std::string pcap_path, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double, asensing_lidar::LidarScanPtr)> pcl_callback, \
      uint16_t start_angle, int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType, // the default timestamp type is LiDAR time
      std::string lidar_correction_file, bool coordinate_correction_flag,
      std::string target_frame, std::string fixed_frame); 
  ~LidarDriverSDK();

  /**
   * @brief load the correction file
   * @param file The path of correction file
   */
  int LoadLidarCorrectionFile(std::string correction_content);//登录校准文件
  void ResetLidarStartAngle(uint16_t start_angle);//读取开始角度
  std::string GetLidarCalibration();//获取标定文件
  void GetCalibrationFromDevice();//获取雷达设备的标定文件
  void Start();
  void Stop();
  void PushScanPacket(asensing_lidar::LidarScanPtr scan);//推送扫描包

 private:
  LidarDriver *lidarDriver;//LidarDriver类型指针
  void *tcp_command_client_;//tcp命令客户端
  boost::thread *get_calibration_thr_;
  bool enable_get_calibration_thr_;
  bool got_lidar_calibration_;
  std::string correction_content_;//标定内容
  std::string correction_file_path_;//标定文件路径
};



#endif  // LIDAR_DRIVER_SDK_H_
