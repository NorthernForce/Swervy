#pragma once

#include "frc/kinematics/SwerveModuleState.h"
#include <ctre/Phoenix.h>
#include <memory>

/**
 * The DriveModule class.
 * Represents a single swerve module on the robot.
 */
class DriveModule {
 public:
  DriveModule(uint8_t driveTalonID, uint8_t turnTalonID, uint8_t canCoderID, double turnOffset);
  void ConfigureDriveTalon();
  void ConfigureTurnTalon();
  void SetState(frc::SwerveModuleState state);
  frc::SwerveModuleState GetState();
  void ResetEncoders();
  void SetDriveTalon(ControlMode controlMode, double value);
  void SetTurnTalon(units::radian_t radians);
  double GetVelocity();
  units::radian_t GetAbsoluteRotation();
  double GetRelativeRotation();
  
 private:
  std::shared_ptr<WPI_TalonFX> driveTalon;
  std::shared_ptr<WPI_TalonFX> turnTalon;
  std::shared_ptr<CANCoder> canCoder;

  const double turnOffset;
};
