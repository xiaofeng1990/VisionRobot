#include "episode.h"

Episode::Episode(/* args */)
{
}

Episode::~Episode()
{
    Disconnect();
}

bool Episode::Connect(const std::string &server_ip, int port) {}
bool Episode::Disconnect() {}
// 急停
json Episode::EmergencyStop(int enable) {}

json Episode::AngleMode(const std::vector<double> &angles, double speed_ratio = 1.0) {}
json Episode::MoveXYZRotation(const std::vector<double> &position, const std::vector<double> &orientation,
                              const std::string &rotation_order = "zyx", double speed_ratio = 1.0) {}
json Episode::MoveLinearXYZRotation(const std::vector<double> &position, const std::vector<double> &orientation,
                                    const std::string &rotation_order = "zyx") {}
// 开启夹爪
json Episode::GripperOn() {}
json Episode::GripperOff() {}
// 夹爪控制
json Episode::ServoGripper(int angle) {}
json Episode::RobodkSimu(int enable) {}

json Episode::SetFreeMode(int mode) {}
json Episode::GetMotorAngles() {}

bool Episode::SendCommand(const std::string &command) {}