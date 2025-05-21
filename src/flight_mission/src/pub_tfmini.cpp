#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/distance.hpp"

#include <termios.h>         // 串口参数配置头文件（termio 结构）
#include <fcntl.h>           // 文件控制头文件（open 函数标志）
#include <unistd.h>          // 系统调用头文件（read/close 函数）
#include <thread>            // 线程库（用于串口读取线程）

using base_interfaces_demo::msg::Distance;

class TFMiniNode : public rclcpp::Node {
public:
    // 构造函数：初始化节点、串口和发布者
    TFMiniNode() : Node("tfmini_node") {  // 节点名称为 "tfmini_node"
        // 1. 创建话题发布者（发布类型为 Distance，话题名 /distance，队列大小 10）
        publisher_ = this->create_publisher<Distance>("/distance", 10);

        // 2. 打开串口设备（假设实际设备为 /dev/ttyUSB1，需根据硬件调整）
        // O_RDWR: 读写模式 | O_NOCTTY: 不将串口作为控制终端 | O_NDELAY: 非阻塞模式（不等待数据）
        serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {  // 检查串口是否成功打开
            RCLCPP_ERROR(this->get_logger(), "打开串口失败，请检查设备路径或权限");
            return;  // 串口打开失败时，节点初始化终止
        }

        // 3. 配置串口参数（波特率 115200，8位数据位，无校验位）
        struct termios tio;  // 串口参数结构体
        tcgetattr(serial_fd_, &tio);  // 获取当前串口参数
        // 配置控制模式（c_cflag）：
        tio.c_cflag = B115200 |  // 波特率 115200
                      CS8 |       // 8位数据位
                      CLOCAL |    // 忽略 modem 控制线（本地连接）
                      CREAD;      // 启用接收功能
        tio.c_iflag = IGNPAR;    // 忽略奇偶校验错误（无校验位）
        tio.c_oflag = 0;         // 输出模式：原始数据输出（不处理）
        tio.c_lflag = 0;         // 本地模式：禁用终端特性（如回显）
        tio.c_cc[VTIME] = 0;     // 读取超时时间（0 表示不等待，单位：0.1秒）
        tio.c_cc[VMIN] = 1;      // 最小读取字节数（至少读取1字节才返回）
        // 立即应用串口参数（TCSANOW: 不等待数据传输完成）
        tcsetattr(serial_fd_, TCSANOW, &tio);

        // 4. 启动串口读取线程（避免阻塞 ROS2 主循环）
        read_thread_ = std::thread([this]() { this->readSerialData(); });
    }

    // 析构函数：释放串口和线程资源
    ~TFMiniNode() {
        // 关闭串口设备
        if (serial_fd_ != -1) {
            close(serial_fd_);
            serial_fd_ = -1;  // 标记为已关闭
        }
        // 等待线程退出（确保资源释放）
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    }

private:
    // 串口数据读取与解析函数（运行在独立线程中）
    void readSerialData() {
        uint8_t recv_buf[9];  // 定义缓冲区（每次读取 9 字节，与 TFMini 协议一致）
        while (rclcpp::ok()) {  // ROS2 节点有效时持续运行
            // 从串口读取数据（返回读取的字节数，-1 表示错误，0 表示无数据）
            int read_count = read(serial_fd_, recv_buf, 9);

            if (read_count == 9) {  // 成功读取 9 字节（符合协议长度）
                // 解析逻辑 1：兼容 Python 3 的 0x59 头格式（十六进制头）
                if (recv_buf[0] == 0x59 && recv_buf[1] == 0x59) {
                    // 距离值：低字节在 recv_buf[2]，高字节在 recv_buf[3]（小端模式）
                    int distance = recv_buf[2] + (recv_buf[3] << 8);

                    // 构造消息并发布
                    auto msg = std::make_unique<Distance>();
                    msg->distance = distance;    // 距离（单位：cm）
                    publisher_->publish(std::move(msg));  // 发布消息
                }
                // 解析逻辑 2：兼容 Python 2 的 'Y' 头格式（字符头）
                else if (recv_buf[0] == 'Y' && recv_buf[1] == 'Y') {
                    // 距离值：低字节 recv_buf[2]，高字节 recv_buf[3]
                    int lowD = static_cast<int>(recv_buf[2]);
                    int highD = static_cast<int>(recv_buf[3]);
                    int distance = lowD + (highD << 8);

                    // 构造并发布消息
                    auto msg = std::make_unique<Distance>();
                    msg->distance = distance;
                    publisher_->publish(std::move(msg));
                }
            }
        }
    }

    int serial_fd_ = -1;  // 串口文件描述符（-1 表示未打开）
    rclcpp::Publisher<Distance>::SharedPtr publisher_;  // ROS2 话题发布者
    std::thread read_thread_;  // 串口读取线程（独立于 ROS2 主循环）
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFMiniNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
