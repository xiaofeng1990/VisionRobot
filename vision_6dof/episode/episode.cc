#include "episode.h"
#include "common/xt_utils.h"

Episode::Episode(/* args */)
{
}

Episode::~Episode()
{
    Disconnect();
}

bool Episode::Connect(const std::string server_ip, int port)
{

    // 创建 socket
    sockfd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd_ == INVALID_SOCKET)
    {
        std::cout << "无法创建 socket" << std::endl;
        return false;
    }

    sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0)
    {
        close(sockfd_);
        std::cout << "无效的 IP 地址" << std::endl;
        return false;
    }

    if (connect(sockfd_, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) == SOCKET_ERROR)
    {
        close(sockfd_);
        std::cout << "连接服务器失败" << std::endl;
        return false;
    }
    return true;
}

void Episode::Disconnect()
{
    if (sockfd_ != -1)
        close(sockfd_);
}
// 急停
json Episode::EmergencyStop(int enable)
{
    json command;
    command["action"] = "emergency_stop";
    command["params"] = enable;
    return SendCommand(command);
}

json Episode::AngleMode(const std::vector<double> &angles, double speed_ratio)
{
    json command;
    command["action"] = "angle_mode";
    command["params"] = {angles, speed_ratio};
    return SendCommand(command);
}
json Episode::MoveXYZRotation(const std::vector<double> &position, const std::vector<double> &orientation,
                              const std::string &rotation_order, double speed_ratio)
{
    json command;
    json params = json::array();
    for (double val : position)
    {
        params.push_back(val);
    }
    for (double val : orientation)
    {
        params.push_back(val);
    }
    params.push_back(rotation_order);
    params.push_back(speed_ratio);
    command["action"] = "move_xyz_rotation";
    command["params"] = params;
    return SendCommand(command);
}
json Episode::MoveLinearXYZRotation(const std::vector<double> &position, const std::vector<double> &orientation,
                                    const std::string &rotation_order)
{
    json command;
    json params = json::array();
    for (double val : position)
    {
        params.push_back(val);
    }
    for (double val : orientation)
    {
        params.push_back(val);
    }
    params.push_back(rotation_order);
    command["action"] = "move_linear_xyz_rotation";
    command["params"] = params;
    return SendCommand(command);
}
// 开启夹爪
json Episode::GripperOn()
{
    json command;
    command["action"] = "gripper_on";
    return SendCommand(command);
}
json Episode::GripperOff()
{
    json command;
    command["action"] = "gripper_off";
    return SendCommand(command);
}
// 夹爪控制
json Episode::ServoGripper(int angle)
{
    json command;
    command["action"] = "servo_gripper";
    command["params"] = angle;
    return SendCommand(command);
}
json Episode::RobodkSimu(int enable)
{
    json command;
    command["action"] = "robodk_simu";
    command["params"] = enable;
    return SendCommand(command);
}

json Episode::SetFreeMode(int mode)
{
    json command;
    command["action"] = "set_free_mode";
    command["params"] = mode;
    return SendCommand(command);
}
json Episode::GetMotorAngles()
{
    json command;
    command["action"] = "get_motor_angles";
    return SendCommand(command);
}

json Episode::GetT()
{
    json command;
    command["action"] = "get_T";
    return SendCommand(command);
}

json Episode::GetPose(const std::string &rotation_order)
{
    json command;
    command["action"] = "get_pose";
    command["params"] = "rotation_order";
    return SendCommand(command);
}

json Episode::SendCommand(const json &command)
{
    if (sockfd_ == -1)
    {
        std::cerr << "Socket is not connected." << std::endl;
        return false;
    }

    // 序列化命令为 JSON 字符串
    std::string data = command.dump();
    uint64_t data_length = data.size();

    // 将数据长度转换为网络字节序（8 字节）
    uint64_t net_length = htonll(data_length);

    // 发送数据长度
    if (send_all(sockfd_, &net_length, sizeof(net_length)) != sizeof(net_length))
    {

        std::cerr << "发送数据长度失败" << std::endl;

        throw std::runtime_error("发送数据长度失败");
    }
    // 发送实际 JSON 数据
    if (send_all(sockfd_, data.c_str(), data_length) != static_cast<ssize_t>(data_length))
    {
        throw std::runtime_error("发送数据失败");
    }

    // 接收响应头：8 字节数据长度
    uint64_t response_length_net = 0;
    if (recv_all(sockfd_, &response_length_net, sizeof(response_length_net)) != sizeof(response_length_net))
    {

        throw std::runtime_error("接收响应长度失败");
    }
    uint64_t response_length = ntohll(response_length_net);

    // 接收响应数据
    std::string response_data;
    response_data.resize(response_length);
    if (recv_all(sockfd_, &response_data[0], response_length) != static_cast<ssize_t>(response_length))
    {

        throw std::runtime_error("接收响应数据失败");
    }

    // 解析 JSON 响应
    json result = json::parse(response_data);
    return result;
}

int Episode::Test()
{
    std::cout << "3dof vision" << std::endl;

    Episode client;
    client.Connect("127.0.0.1", 12345);

    std::cout << "移动到默认位置..." << std::endl;
    json result = client.AngleMode({180.0, 90.0, 83.0, 30.0, 110.0, 30.0}, 1.0);
    double sleep_time = result.get<double>();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

    result = client.GetT();
    std::cout << "get_T 响应: " << result.dump() << std::endl;
    result = client.GetPose("xyz");
    std::cout << "get_pose 响应: " << result.dump() << std::endl;

#if 0
    std::cout << "即将以 0.5 速度移动到指定位置[357.019928, 0.000000, 305.264329]，朝向[90, 0, 180]，旋转顺序 zyx..." << std::endl;
    result = client.MoveXYZRotation({357.019928, 0.0, 305.264329}, {90.0, 0.0, 180.0}, "zyx", 0.5);
    std::cout << "move_xyz_rotation 响应: " << result.dump() << std::endl;
    sleep_time = result.get<double>();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

    std::cout << "即将以 0.5 速度移动到指定位置[386.940216, 180.254456, 74.679009]，朝向[90, 0, 180]，旋转顺序 zyx..." << std::endl;
    result = client.MoveXYZRotation({386.940216, 180.254456, 74.679009}, {90.0, 0.0, 180.0}, "zyx", 0.5);
    std::cout << "move_xyz_rotation 响应: " << result.dump() << std::endl;
    sleep_time = result.get<double>();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

    std::cout << "即将直线移动到指定位置[357.019928, 0.000000, 305.264329]，朝向[90, 0, 180]，旋转顺序 zyx..." << std::endl;
    result = client.MoveLinearXYZRotation({357.019928, 0.0, 305.264329}, {90.0, 0.0, 180.0}, "zyx");
    std::cout << "move_linear_xyz_rotation 响应: " << result.dump() << std::endl;
    sleep_time = result.get<double>();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
#endif
    return 0;
}