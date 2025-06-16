#include <string>
#include <iostream>
#include "orbbec/orbbec.h"
#include "episode/episode.h"
int main()
{
    std::cout << "3dof vision" << std::endl;

    Episode client;
    client.Connect("127.0.0.1", 12345);
    std::cout << "移动到默认位置..." << std::endl;
    json result = client.AngleMode({180.0, 90.0, 83.0, 30.0, 110.0, 30.0}, 1.0);
    double sleep_time = result.get<double>();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

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
    return 0;
}
