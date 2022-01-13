/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc2/command/RunCommand.h>
#include "commands/DriveWithJoystick.h"

std::shared_ptr<OI> RobotContainer::oi;
std::shared_ptr<Drivetrain> RobotContainer::drivetrain;
std::shared_ptr<IMU> RobotContainer::imu;

RobotContainer::RobotContainer() {
    oi.reset(new OI());
    InitSubsystems();
    oi->MapControllerButtons();
    InitDefaultCommands();
}

void RobotContainer::InitSubsystems() {
    drivetrain.reset(new Drivetrain());
    imu.reset(new IMU());
}

void RobotContainer::InitDefaultCommands() {
    drivetrain->SetDefaultCommand(DriveWithJoystick());
}

void RobotContainer::InitAutonomousCommands() {}