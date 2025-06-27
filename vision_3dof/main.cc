#include <string>
#include <iostream>
#include "orbbec/orbbec.h"
#include "episode/episode.h"
#include "calibration/calibration.h"
int main()
{
    std::cout << "3dof vision" << std::endl;

    // Orbbec orbbec;
    // orbbec.TestCamera();

    Calibration cal;
    cal.InitDevice();
    // cal.PerformCalibration();
    std::string file_path = "calibration/affine_matrix.yml";
    // cal.TestCalibration(file_path);
    cal.TestCalibration2(file_path);
    return 0;
}
