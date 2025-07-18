#include <string>
#include <iostream>
#include "orbbec/orbbec.h"
#include "episode/episode.h"
#include "calibration/calibration.h"
#include "teach_mode/teach_mode.h"
int main()
{
    std::cout << "6dof vision" << std::endl;
    Calibration cali;
    cali.InitDevice();
    // cali.PreSample();
    // cali.Sample();
    // cali.PerformCalibration();
    cali.TestCalibration("./calibration/hand_eye_calibration.yml");

    return 0;
}
