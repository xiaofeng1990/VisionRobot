#include <string>
#include <iostream>
#include "orbbec/orbbec.h"
int main()
{
    std::cout << "3dof vision" << std::endl;
    Orbbec orbbec;
    if (!orbbec.GetDevice(0))
    {
        std::cerr << "failed to open device" << std::endl;
        return -1;
    }
    std::string serial_number = "CP3294Y0003N";
    orbbec.OpenDevice(serial_number);
    orbbec.PrintfPropertyList();
    cv::Mat mat;
    orbbec.GetColorFrame(mat);
    cv::imwrite("./color.jpg", mat);
    return 0;
}
