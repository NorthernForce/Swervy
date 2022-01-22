#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>

#include "OI.h"
#include "Constants.h"
#include "commands/DriveWithJoystick.h"
#include "commands/TankDriveForward.h"
#include "commands/ZeroTurnMotors.h"

std::shared_ptr<frc::XboxController> OI::driverController;

OI::OI() {
    frc::SmartDashboard::PutNumber("Drive Speed:", 1.0);  
    InitControllers();
}

void OI::InitControllers() {
    driverController = std::make_shared<frc::XboxController>(Constants::driverControllerID);
}

void OI::MapControllerButtons() {
    SimpleButton(driverController, Xbox::menu_button).WhileHeld(ZeroTurnMotors());
}

std::vector<double> OI::GetRawDriveControls() {
    speed = driverController->GetY(frc::XboxController::JoystickHand::kLeftHand);
    strafe = driverController->GetX(frc::XboxController::JoystickHand::kLeftHand);
    yaw = driverController->GetX(frc::XboxController::JoystickHand::kRightHand);
    return {speed, strafe, yaw};
}

std::vector<double> OI::GetDriveControls() {
    // Scale from -1-1 to user-specified interval, default includes deadband
    std::vector<double> driveControls = this->GetRawDriveControls();
    for (uint8_t i=0; i < 3; i++) {
        double *direction_value = &driveControls[i];
        if (*direction_value > -0.25 && *direction_value < 0.25)
            *direction_value = 0;
    }
    return driveControls;
}

void OI::SetControllerRumble(frc::XboxController *controller, double value, bool lightly) {
  if (lightly)
    controller->SetRumble(frc::GenericHID::RumbleType::kRightRumble, value);
  else
    controller->SetRumble(frc::GenericHID::RumbleType::kLeftRumble, value);
}

double OI::CheckAndLimitValue(double value, double upperLimit, double lowerLimit) {
  if (value < lowerLimit)
    value = lowerLimit;
  else if (value > upperLimit)
    value = upperLimit;
  return value;
}

frc2::Button OI::SimpleButton(std::shared_ptr<frc::GenericHID> controller, uint8_t btn) {
  return frc2::Button([this, controller, btn] { return controller->GetRawButton(btn); });
}

frc2::Button OI::SimpleAxis(std::shared_ptr<frc::GenericHID> controller, uint8_t axis, double threshold) {
    return frc2::Button([this, controller, axis, threshold] {
        if (threshold < 0)
            return controller->GetRawAxis(axis) < threshold;
        else
            return controller->GetRawAxis(axis) > threshold;
    });
}

frc2::POVButton OI::SimplePOV(std::shared_ptr<frc::GenericHID> controller, uint16_t degs) {
    return frc2::POVButton(controller.get(), degs);
}