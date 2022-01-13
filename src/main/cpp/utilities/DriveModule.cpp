#include "utilities/DriveModule.h"

#include "units/math.h"
#include <cmath>
#include <stdio.h>

#define DRIVE_ENC_TO_METERS_FACTOR 0.00002226
#define RAD_TO_ENC_FACTOR 10.1859

#define DRIVE_MAX_VOLTAGE 12
#define ROT_MAX_VOLTAGE 12

#define DRIVE_P_VALUE 1
#define DRIVE_I_VALUE 0
#define DRIVE_D_VALUE 0
#define DRIVE_I_ZONE_VALUE 0
#define DRIVE_FF_VALUE 1023/(DRIVE_MAX_VOLTAGE/DRIVE_ENC_TO_METERS_FACTOR)

#define ROT_P_VALUE 0.4
#define ROT_I_VALUE 0
#define ROT_D_VALUE 0
#define ROT_I_ZONE_VALUE 0
#define ROT_FF_VALUE 0

DriveModule::DriveModule(uint8_t driveTalonID, uint8_t turnTalonID, uint8_t canCoderID, double turnOffset) 
 : turnOffset(turnOffset) 
{
  driveTalon = std::make_shared<WPI_TalonFX>(driveTalonID);
  turnTalon = std::make_shared<WPI_TalonFX>(turnTalonID);
  canCoder = std::make_shared<CANCoder>(canCoderID);

  ConfigureDriveTalon();
  ConfigureTurnTalon();
}

void DriveModule::ConfigureDriveTalon() {
  driveTalon->ConfigFactoryDefault();
  driveTalon->SetNeutralMode(NeutralMode::Brake);
  driveTalon->ConfigVoltageCompSaturation(DRIVE_MAX_VOLTAGE);
  driveTalon->EnableVoltageCompensation(true);
  driveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  driveTalon->ConfigSelectedFeedbackCoefficient(DRIVE_ENC_TO_METERS_FACTOR);
  driveTalon->SetInverted(false);
  driveTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 10);
  driveTalon->SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 10);
  driveTalon->SetSelectedSensorPosition(0);
  driveTalon->Config_kP(0, DRIVE_P_VALUE);
  driveTalon->Config_kI(0, DRIVE_I_VALUE);
  driveTalon->Config_kD(0, DRIVE_D_VALUE);
  driveTalon->Config_IntegralZone(0, DRIVE_I_ZONE_VALUE);
  driveTalon->Config_kF(0, DRIVE_FF_VALUE);
}

void DriveModule::ConfigureTurnTalon() {
  turnTalon->ConfigFactoryDefault();
  turnTalon->SetNeutralMode(NeutralMode::Coast);
  turnTalon->ConfigVoltageCompSaturation(ROT_MAX_VOLTAGE);
  turnTalon->EnableVoltageCompensation(true);
  turnTalon->SetInverted(true);
  turnTalon->ConfigRemoteFeedbackFilter(*canCoder.get(), 0);
  turnTalon->Config_kP(0, ROT_P_VALUE);
  turnTalon->Config_kI(0, ROT_I_VALUE);
  turnTalon->Config_kD(0, ROT_D_VALUE);
  turnTalon->Config_IntegralZone(0, ROT_I_ZONE_VALUE);
  turnTalon->Config_kF(0, ROT_FF_VALUE);
}
 
void DriveModule::SetState(frc::SwerveModuleState targetState) {
  frc::SwerveModuleState currentState = GetState();
  frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(targetState, currentState.angle);
  
  if (units::math::abs(optimizedState.speed) > 0.01_mps)
    SetTurnTalon(optimizedState.angle.Radians());  

  SetDriveTalon(ControlMode::PercentOutput, optimizedState.speed.value());
}

frc::SwerveModuleState DriveModule::GetState() {
  return { units::meters_per_second_t(GetVelocity()), frc::Rotation2d(GetAbsoluteRotation()) };
}

void DriveModule::ResetEncoders() {
  CANCoderConfiguration config;
  canCoder->GetAllConfigs(config);
  canCoder->ConfigMagnetOffset(config.magnetOffsetDegrees - canCoder->GetAbsolutePosition());
}

void DriveModule::SetDriveTalon(ControlMode controlMode, double value) {
  driveTalon->Set(controlMode, value);
}

void DriveModule::SetTurnTalon(units::radian_t radians) {
  units::radian_t rotation(radians - GetAbsoluteRotation());
  units::radian_t absRotation(units::math::abs(rotation));
  
  // Fix the discontinuity problem.
  // If the value is above π rad or below -π rad...
  if(absRotation.value() > wpi::math::pi)
    // Subtract 2π rad, or add 2π rad depending on the sign.
    rotation = units::radian_t(rotation.value() - (2 * wpi::math::pi) * (std::signbit(rotation.value()) ? -1 : 1));
  
  double output = rotation.value() * RAD_TO_ENC_FACTOR;
  output += GetRelativeRotation();
}

double DriveModule::GetVelocity() {
  return driveTalon->GetSensorCollection().GetIntegratedSensorVelocity();
}

units::radian_t DriveModule::GetAbsoluteRotation() {
  return units::radian_t(units::degree_t(canCoder->GetAbsolutePosition()));
}

double DriveModule::GetRelativeRotation() {
  return turnTalon->GetSensorCollection().GetIntegratedSensorPosition() / 12.8; // gear ratio offset
}
