
#include "RobotContainer.h"
#include "Constants.h"

#include "frc/kinematics/SwerveModuleState.h"
#include <cmath>
#include <stdio.h>

Drivetrain::Drivetrain() {
  RobotContainer::imu->Reset();

  swerveModules = {
    std::make_shared<DriveModule>(Constants::MotorIDs::driveFL, Constants::MotorIDs::turnFL, Constants::EncoderIDs::encoderFL, -0.09),
    std::make_shared<DriveModule>(Constants::MotorIDs::driveRL, Constants::MotorIDs::turnRL, Constants::EncoderIDs::encoderRL, +0.29),
    std::make_shared<DriveModule>(Constants::MotorIDs::driveRR, Constants::MotorIDs::turnRR, Constants::EncoderIDs::encoderRR, +1.25),
    std::make_shared<DriveModule>(Constants::MotorIDs::driveFR, Constants::MotorIDs::turnFR, Constants::EncoderIDs::encoderFR, +1.35)
  };
}

void Drivetrain::SetDrive(frc::ChassisSpeeds chassisSpeeds, units::meters_per_second_t _maxSpeed) {
  // Get target states.
  wpi::array<frc::SwerveModuleState, 4> moduleStates = kinematics.ToSwerveModuleStates(chassisSpeeds);
  kinematics.NormalizeWheelSpeeds(&moduleStates, _maxSpeed);

  for (uint8_t i = 0; i < swerveModules.size(); i++) {
    swerveModules.at(i)->SetState(moduleStates.at(i));
  }
}

void Drivetrain::SetDrive(units::velocity::meters_per_second_t xVelMeters,
                     units::velocity::meters_per_second_t yVelMeters,
                     units::degrees_per_second_t degreesPerSecond,
                     bool isFieldCentric) {
  if(isFieldCentric)
    // Create relative chassis speeds struct from parameters, then call the other setDrive function.
    SetDrive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(xVelMeters, yVelMeters, units::radians_per_second_t(degreesPerSecond), GetRotation()), maxSpeed);
  else
    // Create chassis speeds struct from parameters, then call the other setDrive function.
    SetDrive({ xVelMeters, yVelMeters, units::radians_per_second_t(degreesPerSecond) }, maxSpeed);
}

void Drivetrain::Process() {
  UpdateOdometry();

  if (!cmdRunning) return;

  frc::Pose2d currentPos = GetPose();
  // TODO Drive
}

frc::Rotation2d Drivetrain::GetRotation() {
  double angle = std::fmod(RobotContainer::imu->GetRotation(), 360);

  double rotation = 0;
  if (abs(angle) > 180)
    rotation = angle - 360 * (std::signbit(angle) ? -1 : 1);
  else
    rotation = angle;
  
  return frc::Rotation2d(units::degree_t(rotation));
}

void Drivetrain::UpdateOdometry() {
  odometry.Update(GetRotation(),
    swerveModules.at(0)->GetState(),
    swerveModules.at(1)->GetState(),
    swerveModules.at(2)->GetState(),
    swerveModules.at(3)->GetState());
}

void Drivetrain::ResetOdometry(frc::Pose2d pose) {
  odometry.ResetPosition(pose, GetRotation());
}

frc::Pose2d Drivetrain::GetPose() {
  return odometry.GetPose();
}

void Drivetrain::CmdGoToPose(frc::Pose2d pose, units::meters_per_second_t speed) {
  if (!LastCmdIsFinished()) {
    CmdCancel();
  }
  cmdRunning = true;
  cmdPose = pose;
  cmdSpeed = speed;
}

void Drivetrain::CmdRotate(units::degree_t angle, units::meters_per_second_t speed) {
  if (!LastCmdIsFinished()) {
    CmdCancel();
  }
  cmdRunning = true;  
  frc::Pose2d currentPos = GetPose();
  cmdPose = {currentPos.X(), currentPos.Y(), (currentPos.Rotation().Degrees() + angle)};
  cmdSpeed = speed;
}

void Drivetrain::CmdDrive(units::meter_t amount, units::degree_t angle, units::meters_per_second_t speed) {
  if (!LastCmdIsFinished()) {
    CmdCancel();
  }
  // TODO Implement.
}

bool Drivetrain::LastCmdIsFinished() {
  return cmdRunning;
}

void Drivetrain::CmdCancel() {
  cmdRunning = false;
}

void Drivetrain::ResetSwerveEncoders() {
  for (auto module : swerveModules)
    module->ResetEncoders();
}

void Drivetrain::StopAll() {
    CmdCancel();
    for (std::shared_ptr<DriveModule> module : swerveModules) {
      module->SetDriveTalon(ControlMode::PercentOutput, 0);
      module->SetDriveTalon(ControlMode::PercentOutput, 0);
    }
}