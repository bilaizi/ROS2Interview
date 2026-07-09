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
