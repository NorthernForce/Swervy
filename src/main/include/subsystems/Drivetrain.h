#pragma once

#include "utilities/DriveModule.h"
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"

#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "wpi/array.h"
#include "wpi/math"

#define ROBOT_WIDTH 0.362_m
#define ROBOT_LENGTH 0.66_m

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();

  void SetDrive(frc::ChassisSpeeds chassisSpeeds, units::meters_per_second_t maxSpeed);
  void SetDrive(units::velocity::meters_per_second_t xVelMeters,
                units::velocity::meters_per_second_t yVelMeters,
                units::degrees_per_second_t degreesPerSecond,
                bool isFieldCentric = false);
  
  static constexpr units::meters_per_second_t maxSpeed = 4_mps;

  void Process();
  frc::Rotation2d GetRotation();
  void UpdateOdometry();
  void ResetOdometry(frc::Pose2d pose);
  frc::Pose2d GetPose();
  void CmdGoToPose(frc::Pose2d pose, units::meters_per_second_t speed = maxSpeed/2);
  void CmdRotate(units::degree_t angle, units::meters_per_second_t speed = maxSpeed/2);
  void CmdDrive(units::meter_t amount, units::degree_t angle = units::degree_t(0), units::meters_per_second_t speed = maxSpeed/2);
  bool LastCmdIsFinished();
  void CmdCancel();
  void ResetSwerveEncoders();
  void StopAll();

  /**
   * The locations of the swerve modules on the robot.
   *
   * Swerve Module Arrangement:
   *           0 2
   *           1 3
   */
  wpi::array<frc::Translation2d, 4> locations {
    frc::Translation2d(-ROBOT_WIDTH/2, +ROBOT_LENGTH/2),
    frc::Translation2d(-ROBOT_WIDTH/2, -ROBOT_LENGTH/2),
    frc::Translation2d(+ROBOT_WIDTH/2, +ROBOT_LENGTH/2),
    frc::Translation2d(+ROBOT_WIDTH/2, -ROBOT_LENGTH/2),
  };

  wpi::array<std::shared_ptr<DriveModule>, 4> swerveModules {
    std::make_shared<DriveModule>(Constants::MotorIDs::driveFL, Constants::MotorIDs::turnFL, Constants::EncoderIDs::encoderFL, -0.09),
    std::make_shared<DriveModule>(Constants::MotorIDs::driveRL, Constants::MotorIDs::turnRL, Constants::EncoderIDs::encoderRL, +0.29),
    std::make_shared<DriveModule>(Constants::MotorIDs::driveRR, Constants::MotorIDs::turnRR, Constants::EncoderIDs::encoderRR, +1.25),
    std::make_shared<DriveModule>(Constants::MotorIDs::driveFR, Constants::MotorIDs::turnFR, Constants::EncoderIDs::encoderFR, +1.35),
  };

  //Class converts chassis speeds into swerve module states.
  frc::SwerveDriveKinematics<4> kinematics { locations };

  //Handles field-centric stuff.
  frc::SwerveDriveOdometry<4> odometry { kinematics, GetRotation() };

 private:
  bool cmdRunning = false;
  frc::Pose2d cmdPose;
  units::meters_per_second_t cmdSpeed;
};
