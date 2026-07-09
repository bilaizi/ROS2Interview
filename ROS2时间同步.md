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
