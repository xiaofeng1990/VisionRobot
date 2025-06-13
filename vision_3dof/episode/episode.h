#include "json.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <chrono>
#include <thread>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR (-1)
typedef int SOCKET;
using json = nlohmann::json;
class Episode
{
public:
    Episode();
    ~Episode();
    bool Connect(const std::string &server_ip, int port);
    void Disconnect();
    // 急停
    json EmergencyStop(int enable);

    json AngleMode(const std::vector<double> &angles, double speed_ratio = 1.0);
    json MoveXYZRotation(const std::vector<double> &position, const std::vector<double> &orientation,
                         const std::string &rotation_order = "zyx", double speed_ratio = 1.0);
    json MoveLinearXYZRotation(const std::vector<double> &position, const std::vector<double> &orientation,
                               const std::string &rotation_order = "zyx");
    // 开启夹爪
    json GripperOn();
    json GripperOff();
    // 夹爪控制
    json ServoGripper(int angle);
    json RobodkSimu(int enable);

    json SetFreeMode(int mode);
    json GetMotorAngles();

private:
    json SendCommand(const json &command);

private:
    int port_;
    std::string server_ip_;
    int sockfd_;
};
