练习 9.1：一个机器人系统有激光雷达（10Hz）、相机（30Hz）和 IMU（200Hz），如何设计时间同步策略确保传感器数据融合的准确性？
提示：考虑硬件触发、时间戳插值和数据缓冲。
参考答案
时间同步策略设计： 
  1. 硬件层同步： - 使用 GPS PPS 信号作为统一触发源 - 通过 FPGA 分频产生各传感器触发信号 - 激光雷达：100ms 触发一次 - 相机：33.3ms 触发一次 - IMU：5ms 触发一次 
  2. 软件层处理： - 为每个传感器数据附加硬件时间戳 - 使用环形缓冲区存储最近 1 秒的数据 - 以激光雷达时间为基准（最低频率） - 相机数据：选择时间戳最近的帧 - IMU 数据：进行插值或积分到目标时间
  3. 数据对齐算法： - 设置时间容差窗口（如 ±5ms） - 实现时间戳排序队列 - 当激光雷达数据到达时，查找窗口内的相机和 IMU 数据 - 使用插值算法对齐到统一时间点
练习 9.2：计算 rosbag2 录制 1 小时数据的存储需求。假设有 5 个 10Hz 的激光雷达话题（每帧 2MB）、10 个 30Hz 的相机话题（每帧 1MB）、1 个 100Hz 的控制指令话题（每条 1KB）。
提示：考虑压缩比和元数据开销。
参考答案
存储需求计算： 1. 原始数据量：
                  - 激光雷达：5 × 10Hz × 2MB × 3600s = 360GB
                  - 相机：10 × 30Hz × 1MB × 3600s = 1080GB
                  - 控制指令：1 × 100Hz × 1KB × 3600s = 360MB
                  - 总计：约 1440.36GB
              2. 考虑压缩（LZ4，压缩比 12:1）： - 压缩后：1440.36GB / 12 = 120.03GB
              3. 元数据开销（约 5%）： - 最终存储：120.03GB × 1.05 = 126.03GB
              4. 实际考虑： - 使用分片存储，每个文件 4GB - 需要约 32 个数据文件 - 预留 20% 缓冲空间 - 建议准备 150GB 存储空间


🔹 通用数据结构（C++）
struct SyncFrame {
    std::vector<uint8_t> data;
    int width, height;
    long long timestamp_us; // 微秒级时间戳
    std::string camera_id;
};

// 每台相机一个队列
std::map<std::string, RingBuffer<SyncFrame>> g_cameraBuffers;
std::mutex g_alignMutex;

1. 海康（Hikvision）MV-CH 系列
✅ C++（硬件触发 + 高精度时间戳）
// 初始化相机（以相机1为例）
MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_ON);
MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0); // 外部触发
MV_CC_StartGrabbing(handle);

// 回调函数
void __stdcall OnImage(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pInfo, void* pUser) {
    auto frame = std::make_shared<SyncFrame>();
    frame->data.assign(pData, pData + pInfo->nFrameLen);
    frame->width = pInfo->nWidth;
    frame->height = pInfo->nHeight;
    
    // 海康时间戳：32位高 + 32位低 → 64位纳秒（需转微秒）
    uint64_t ts_ns = ((uint64_t)pInfo->nTimeStampHigh << 32) | pInfo->nTimeStampLow;
    frame->timestamp_us = ts_ns / 1000; // 转微秒
    
    frame->camera_id = *(std::string*)pUser;
    
    {
        std::lock_guard<std::mutex> lock(g_alignMutex);
        g_cameraBuffers[frame->camera_id].Push(*frame);
    }
}
// 注册回调
std::string cam_id = "Hik_01";
MV_CC_RegisterImageCallBack(handle, OnImage, &cam_id);

🔹 2. Basler ace 系列（GigE Vision）
✅ C++（pylon + PTP 时间戳）
// 启用 PTP（若支持）
// camera.GetDeviceInfo().GetPropertyValue("GevIEEE1588") == "Yes"

// 在 GrabLoop 中
CGrabResultPtr ptrGrabResult;
camera.RetrieveResult(1000, ptrGrabResult);

if (ptrGrabResult->GrabSucceeded()) {
    auto frame = std::make_shared<SyncFrame>();
    frame->data.assign((uint8_t*)ptrGrabResult->GetBuffer(), 
                       (uint8_t*)ptrGrabResult->GetBuffer() + ptrGrabResult->GetImageSize());
    frame->width = ptrGrabResult->GetWidth();
    frame->height = ptrGrabResult->GetHeight();
    
    // Basler 时间戳单位：纳秒
    frame->timestamp_us = ptrGrabResult->GetTimeStamp() / 1000;
    frame->camera_id = "Basler_01";
    
    // 入队...
}

 3. 堡盟（Baumer）TXG 系列
✅ C++（BGAPI2 + GenICam 时间戳）
void OnImage(BGAPI2::Image* pImage) {
    auto frame = std::make_shared<SyncFrame>();
    frame->data.resize(pImage->GetBufferSize());
    memcpy(frame->data.data(), pImage->GetBuffer(), pImage->GetBufferSize());
    
    frame->width = pImage->GetWidth();
    frame->height = pImage->GetHeight();
    
    // Baumer 时间戳：微秒（直接可用）
    frame->timestamp_us = pImage->GetTimestamp();
    frame->camera_id = "Baumer_01";
    
    // 入队...
}

时间戳对齐算法（核心！）
// 寻找最近同步帧组
bool FindSyncGroup(std::map<std::string, SyncFrame>& out_group, int tolerance_us = 1000) {
    std::lock_guard<std::mutex> lock(g_alignMutex);
    
    // 获取每台相机最新帧
    std::map<std::string, SyncFrame> latest;
    for (auto& [id, buf] : g_cameraBuffers) {
        if (!buf.Empty()) {
            latest[id] = buf.Front(); // 假设 Front 是最新
        }
    }
    
    if (latest.size() != g_cameraBuffers.size()) return false;
    
    // 找时间戳中位数
    std::vector<long long> timestamps;
    for (auto& [id, frame] : latest) timestamps.push_back(frame.timestamp_us);
    std::sort(timestamps.begin(), timestamps.end());
    long long median_ts = timestamps[timestamps.size() / 2];
    
    // 检查所有帧是否在容忍窗口内
    for (auto& [id, frame] : latest) {
        if (std::abs(frame.timestamp_us - median_ts) > tolerance_us) {
            return false; // 有帧超时，等待下一组
        }
    }
    
    out_group = latest;
    
    // 从队列中移除已对齐帧
    for (auto& [id, _] : out_group) {
        g_cameraBuffers[id].Pop();
    }
    
    return true;
}

// 对齐线程
void AlignThread() {
    while (!g_stop) {
        std::map<std::string, SyncFrame> group;
        if (FindSyncGroup(group)) {
            // 送入处理 pipeline：拼接 / 3D / AI
            ProcessSyncGroup(group);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}
真正的同步 = 硬件保曝光 + 软件保帧序
https://blog.csdn.net/weixin_28224217/article/details/159226787
Livox MID-70 + ROS2 + fast-lio 实战避坑与时间同步精解
