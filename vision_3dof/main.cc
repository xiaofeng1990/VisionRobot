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
    return 0;
}
