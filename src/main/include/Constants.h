// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <stdint.h>

namespace Constants {
    namespace MotorIDs {
        // first letter: front/rear
        // second letter: left/right
        const uint8_t driveFL = 1;
        const uint8_t turnFL = 2;

        const uint8_t driveFR = 3;
        const uint8_t turnFR = 4;

        const uint8_t driveRL = 5;
        const uint8_t turnRL = 6;

        const uint8_t driveRR = 7;
        const uint8_t turnRR = 8;
    }

    const uint8_t driverControllerID = 0;
}