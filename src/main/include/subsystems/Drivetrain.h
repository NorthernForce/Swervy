// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/RobotDriveBase.h>
#include "utilities/DriveModule.h"

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();

  void Periodic() override;
  void SetDriveSpeed(double fl, double fr, double rl, double rr);
  void SetTurnSpeed(double fl, double fr, double rl, double rr);
  void SetLocation(double fl, double fr, double rl, double rr);
  void StopDrive();
  void StopAll();
  void ResetEncoders();
  void SetAllDriveSpeed(double speed);
  void SetAllTurnSpeed(double speed);
  void SetAllLocation(double loc);
  void SetAllPosition(double pos);

  void SwerveDrive(double fwd, double str, double rot);
  void HumanDrive(double fwd, double str, double rot);
  void TankDrive(double left, double right);

  double GetAngleToLocation(double loc);
  void SetDriveBrakeMode(bool brake);

 private:
    std::shared_ptr<DriveModule> driveModuleFL;
    std::shared_ptr<DriveModule> driveModuleFR;
    std::shared_ptr<DriveModule> driveModuleRL;
    std::shared_ptr<DriveModule> driveModuleRR;

    // double p = 4.2;
    // double i = 0.01;
    // double d = 0;
    // double iZ = 200;

    double p = 1;
    double i = 0;
    double d = 0;
    double iZ = 0;

    const double l = 29;
    const double w = 29;
    const double r = sqrt((l * l) + (w * w));
    uint8_t loopCycleCounter = 0;
};
