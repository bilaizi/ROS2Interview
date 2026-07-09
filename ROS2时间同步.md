https://zhuanlan.zhihu.com/p/1949545679325201011
ROS2时间同步----使用message_filters进行时间软同步
1 message_filter
功能介绍
message_filters 是 ROS2 中提供的一组工具，用于多话题消息同步。
它的核心是 Synchronizer，支持两种常见策略：
ExactTime：严格时间戳相同才触发回调。
ApproximateTime：时间戳近似匹配即可触发，更适合实际传感器场景。
下面的例子订阅了两个话题：/imu（IMU 数据）和 /fix（GPS 数据），并使用 ExactTime 策略进行同步：
      #include <message_filters/subscriber.h>
      #include <message_filters/synchronizer.h>
      #include <message_filters/sync_policies/exact_time.h>
      #include <message_filters/sync_policies/approximate_time.h>
      #include <sensor_msgs/msg/imu.hpp>
      #include <sensor_msgs/msg/nav_sat_fix.hpp>
      
      using namespace sensor_msgs;
      using namespace message_filters;
      
      // 回调函数：收到同步后的 IMU + GPS 数据
      void callback(const Imu::ConstSharedPtr& imu, const NavSatFix::ConstSharedPtr& gps) 
      {
          std::cout << "同步数据: IMU加速度X=" << imu->linear_acceleration.x 
                    << ", GPS纬度=" << gps->latitude << std::endl;
      }
      
      int main(int argc, char** argv) {
          rclcpp::init(argc, argv);
          auto node = std::make_shared<rclcpp::Node>("sync_node");
      
          // 订阅 IMU 和 GPS 话题
          message_filters::Subscriber<Imu> imu_sub(node, "/imu", 1);
          message_filters::Subscriber<NavSatFix> gps_sub(node, "/fix", 1);
      
          // 使用 ExactTime 策略，也可以替换为 ApproximateTime
          typedef sync_policies::ExactTime<Imu, NavSatFix> SyncPolicy;
          message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), imu_sub, gps_sub);
          sync.registerCallback(&callback);
      
          rclcpp::spin(node);
          rclcpp::shutdown();
          return 0;
      }
2 手写同步器
      message_filters缺陷
            编译期固定数量
            需要在模板参数里写死消息数量（如 ExactTime<Imu, NavSatFix>）。
            如果你要同步 N 个话题，就必须修改模板代码并重新编译。
            扩展性差
            无法灵活支持数量不定的消息源。
            在多传感器（如 5 个相机 + 2 个雷达）的情况下，使用起来非常麻烦。
            因此，可以写一个通用的同步器，在运行时动态注册任意数量的传感器，按时间戳进行近似对齐。
      核心思路
            外部注册时间戳提取回调函数。因此该程序不仅可用于ros程序，还可以用于其他通讯架构，如dds等。
            两个核心参数：缓存大小（queue_size）和时间窗口（tolerance）。与message_filters一致。
            使用dfs算法，搜索窗口内最优组合。
            可动态调整改变某一同步主题的valid属性。
            #include <message_filters/subscriber.h>
            #include <message_filters/synchronizer.h>
            #include <message_filters/sync_policies/approximate_time.h>
            #include "sensor_msgs/msg/image.hpp"
            #include "sensor_msgs/msg/point_cloud2.hpp"
            void callback(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                          const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud) {
                std::cout << "同步数据: Image时间=" << img->header.stamp.sec
                          << ", PointCloud时间=" << cloud->header.stamp.sec << std::endl;
            }
            int main(int argc, char** argv) {
                rclcpp::init(argc, argv);
                auto node = rclcpp::Node::make_shared("sync_node");
                message_filters::Subscriber<sensor_msgs::msg::Image> img_sub(node, "/camera/image", 1);
                message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub(node, "/lidar/points", 1);
                typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> MySyncPolicy;
                message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_sub, cloud_sub);
                sync.registerCallback(&callback);
                rclcpp::spin(node);
                rclcpp::shutdown();
                return 0;
            }  

https://zsc.github.io/ros2_tutorial/chapter9.html    
9.1.1 时间概念基础
ROS2 中存在三种基本的时间概念，每种都有其特定的应用场景：
┌─────────────────────────────────────────────────────────┐
│                    ROS2 时间体系                          │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  System Time ──────> 系统墙钟时间（Wall Clock）           │
│       │              始终向前推进                         │
│       │              受系统时间调整影响                    │
│       │                                                 │
│  ROS Time ─────────> ROS 逻辑时间                       │
│       │              可暂停、加速、减速                    │
│       │              用于仿真和回放                       │
│       │                                                 │
│  Steady Time ──────> 单调递增时间                        │
│                      不受系统时间调整影响                  │
│                      用于测量时间间隔                      │
│                                                         │
└─────────────────────────────────────────────────────────
9.1.2 时间戳表示
ROS2 使用 builtin_interfaces::msg::Time 消息类型表示时间戳：
      sec: int32    # 秒部分
      nanosec: uint32  # 纳秒部分 (0-999999999)
这种表示方式提供了纳秒级精度，总时间计算公式为： \(T_{total} = sec + \frac{nanosec}{10^9}\)
9.1.3 时间 API 架构
ROS2 提供了分层的时间 API 架构：

┌──────────────────────────────────────────────┐
│           Application Layer                  │
│         rclcpp::Time, rclpy.Time             │
└────────────────┬─────────────────────────────┘
                 │
┌────────────────▼─────────────────────────────┐
│            RCL Layer                         │
│         rcl_clock_t, rcl_time_t              │
└────────────────┬─────────────────────────────┘
                 │
┌────────────────▼─────────────────────────────┐
│           RMW Layer                          │
│    DDS Timestamp (DDS::Time_t)               │
└────────────────┬─────────────────────────────┘
                 │
┌────────────────▼─────────────────────────────┐
│         Operating System                     │
│    clock_gettime(), CLOCK_REALTIME           │
└──────────────────────────────────────────────┘
9.1.4 时间跳变处理
ROS2 支持时间跳变（Time Jump）检测和处理，这在系统时间调整或仿真时间重置时至关重要：
      时间跳变的数学定义： \(\Delta t = t_{new} - t_{old}\)
      当 $	\Delta t	> threshold$ 时，触发时间跳变回调。跳变分为三种类型：
      向前跳变（Forward Jump）：$\Delta t > 0$
      向后跳变（Backward Jump）：$\Delta t < 0$
      时钟变更（Clock Change）：时钟源类型改变
9.2 时钟源管理
9.2.1 时钟源类型
ROS2 支持多种时钟源，每种都有特定的使用场景：
时钟源类型枚举：
┌────────────────────────────────────────────────┐
│ RCL_SYSTEM_TIME     = 1  # 系统时钟            │
│ RCL_STEADY_TIME     = 2  # 单调时钟            │
│ RCL_ROS_TIME        = 3  # ROS 时钟（默认）     │
│ RCL_EXTERNAL_TIME   = 4  # 外部时钟源          │
└────────────────────────────────────────────────┘
9.2.2 时钟同步机制
多节点间的时钟同步通过 /clock 话题实现：
时钟同步架构：
                    ┌──────────────┐
                    │  Clock Server│
                    │   (仿真器)    │
                    └──────┬───────┘
                           │
                    发布 /clock 话题
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐      ┌──────▼──────┐    ┌─────▼─────┐
   │ Node A  │      │   Node B    │    │  Node C   │
   │订阅/clock│      │ 订阅/clock  │    │订阅/clock │
   └─────────┘      └─────────────┘    └───────────┘
时钟同步的关键参数：    
      发布频率：典型值 100Hz - 1000Hz
      时间容差：允许的最大时间偏差，通常设置为 $\pm 10ms$
      同步延迟：网络传输和处理延迟，需要补偿
9.2.3 时钟源优先级
当存在多个时钟源时，ROS2 采用优先级机制：
      命令行参数覆盖：--ros-args --use-sim-time
      参数服务器配置：use_sim_time 参数
      节点默认设置：代码中指定的时钟类型
      系统默认：RCL_SYSTEM_TIME
9.2.4 时间源质量评估
评估时钟源质量的关键指标：
\[\text{时钟偏差} = \frac{1}{N}\sum_{i=1}^{N}(t_{local,i} - t_{ref,i})\] \[\text{时钟漂移率} = \frac{d(\Delta t)}{dt}\] \[\text{Allan方差} = \frac{1}{2}\langle(x_{n+1} - x_n)^2\rangle\]

9.5.2 技术挑战与解决方案
挑战 1：多传感器时间同步
激光雷达以 10Hz 旋转，相机以 30Hz 拍摄，雷达以 20Hz 扫描，如何确保数据时间对齐？
解决方案：
      硬件层：PTP (IEEE 1588) 时间同步，精度达到亚微秒级
      软件层：基于硬件时间戳的插值对齐算法
      时间对齐算法：
      def align_sensor_data(lidar_t, camera_t, radar_t):
          # 选择激光雷达时间为基准
          base_time = lidar_t
          
          # 相机数据插值
          camera_aligned = interpolate_camera(
              camera_data, camera_t, base_time
          )
          
          # 雷达数据最近邻匹配
          radar_aligned = nearest_neighbor(
              radar_data, radar_t, base_time
          )
          
          return fused_frame
挑战 2：高带宽数据流管理
单车数据率峰值可达 4GB/s，如何无损记录？
解决方案：
      分级存储：热数据在 RAM，温数据在 NVMe，冷数据压缩存储
      智能降采样：根据场景复杂度动态调整采样率
      优先级队列：关键传感器数据优先保证
      数据流量估算： \(\text{总带宽} = \sum_{i=1}^{n} f_i \times s_i \times c_i\)
其中：
      $f_i$：传感器 i 的采样频率
      $s_i$：单次采样数据大小
      $c_i$：压缩比倒数
      挑战 3：确定性回放
如何保证回放时系统行为与实车一致？
解决方案：
      完整状态记录：不仅记录传感器数据，还记录中间计算结果
      依赖注入：回放时注入记录的随机数种子、系统时钟等
      分层验证：每个模块独立验证输出一致性
      
9.6 高级话题
9.6.1 PTP (Precision Time Protocol) 集成
PTP 提供亚微秒级的网络时间同步，是高精度机器人系统的基础。
PTP 工作原理：
主从时钟同步过程：
Master                          Slave
  │                               │
  │──────── Sync (t1) ──────────►│ 记录接收时间 t2
  │                               │
  │──── Follow_Up (t1) ─────────►│
  │                               │
  │◄──── Delay_Req ──────────────│ 发送时间 t3
  │ 记录接收时间 t4              │
  │                               │
  │──── Delay_Resp (t4) ────────►│
  │                               │

时钟偏差计算：
offset = ((t2-t1) - (t4-t3))/2
delay = ((t2-t1) + (t4-t3))/2
ROS2 中集成 PTP：
硬件配置：
# 启用硬件时间戳
sudo ethtool -T eth0
# 配置 PTP 守护进程
sudo ptp4l -i eth0 -m -S
软件集成：
class PTPTimeSource : public rclcpp::Node {
 void sync_callback() {
     // 读取 PTP 时钟
     struct timespec ptp_time;
     clock_gettime(CLOCK_REALTIME, &ptp_time);
        
     // 发布到 /clock 话题
     rosgraph_msgs::msg::Clock clock_msg;
     clock_msg.clock.sec = ptp_time.tv_sec;
     clock_msg.clock.nanosec = ptp_time.tv_nsec;
     clock_pub_->publish(clock_msg);
 }
};
9.6.2 NTP 优化策略
对于精度要求较低（毫秒级）的应用，NTP 是更简单的选择。
NTP 层级架构：
Stratum 0: 原子钟/GPS
        │
Stratum 1: 直连时间源
        │
Stratum 2: 二级服务器  ←── 机器人系统典型层级
        │
Stratum 3: 客户端
Chrony vs NTPd 对比：
特性	Chrony	NTPd
收敛速度	快（分钟级）	慢（小时级）
精度	±1ms	±10ms
资源占用	低	中
间歇网络	支持好	支持差
9.6.3 硬件时间戳技术
网卡硬件时间戳：
      // 启用 SO_TIMESTAMPING
      int flags = SOF_TIMESTAMPING_TX_HARDWARE |
                  SOF_TIMESTAMPING_RX_HARDWARE |
                  SOF_TIMESTAMPING_RAW_HARDWARE;
      setsockopt(sock, SOL_SOCKET, SO_TIMESTAMPING,
                 &flags, sizeof(flags));
传感器硬件触发：
      触发链路：
      GPS PPS 信号 ──► FPGA ──► 触发信号分配
                                 │ │ │
                                 ▼ ▼ ▼
                            相机 激光雷达 IMU
                                 │ │ │
                                 ▼ ▼ ▼
                          硬件时间戳（相同时基）
9.6.4 时间同步监控
实时监控时间同步质量的关键指标：
class TimeSyncMonitor:
    def compute_metrics(self):
        # 时钟偏差
        offset = np.mean(self.offsets)
        
        # 时钟漂移
        drift = np.polyfit(self.timestamps, 
                           self.offsets, 1)[0]
        
        # Allan 偏差（稳定性指标）
        allan_dev = self.allan_deviation(
            self.offsets, self.sample_rate
        )
        
        # 最大时间间隔误差 (MTIE)
        mtie = np.max(self.offsets) - np.min(self.offsets)
        
        return {
            'offset': offset,
            'drift': drift,
            'allan_dev': allan_dev,
            'mtie': mtie
        }
9.6.5 分布式时间同步
大规模多机器人系统的时间同步策略：
层次化同步树：
                 GPS/原子钟
                     │
              ┌──────┴──────┐
              │  主控节点    │
              └──────┬──────┘
                     │
        ┌────────────┼────────────┐
        │            │            │
   ┌────▼───┐   ┌───▼────┐   ┌───▼───┐
   │区域主机│    │区域主机 │   │区域主机│
   └────┬───┘   └────┬────┘   └───┬───┘
        │            │             │
    机器人群      机器人群       机器人群
共识算法时间同步： 基于 Berkeley 算法的分布式时间同步：
\[t_{avg} = \frac{1}{n}\sum_{i=1}^{n}(t_i + \delta_i)\]
其中 $\delta_i$ 是节点 i 到协调者的网络延迟。

9.6.6 时间安全性
防止时间攻击的安全措施：
NTS (Network Time Security)：
      认证的 NTP 协议
      防止中间人攻击
      加密时间戳传输
时间跳变检测：
      bool detect_time_attack(double new_time, double old_time) {
             double jump = new_time - old_time;
             // 检测异常大的时间跳变
             if (abs(jump) > MAX_ALLOWED_JUMP) {
                 log_security_event("Potential time attack detected");
                 return true;
             }
             // 检测时间回退
             if (jump < 0 && !allow_backwards) {
                 return true;
             }
             return false;
      }
