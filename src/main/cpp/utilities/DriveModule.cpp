#include "utilities/DriveModule.h"

DriveModule::DriveModule(uint8_t driveTalonID, uint8_t turnTalonID, double tP, double tI, double tD, int tIZone) {
    driveTalon = std::make_shared<WPI_TalonFX>(driveTalonID);
    turnTalon = std::make_shared<WPI_TalonFX>(turnTalonID);
}

void DriveModule::SetDriveSpeed(double speed) {
    driveTalon->Set(speed);
}

void DriveModule::SetTurnSpeed(double speed) {
    turnTalon->Set(speed);
}

void DriveModule::SetTurnLocation(double loc) {
    double base = GetTurnEncPosition() * encoder_cpr;
    if (GetTurnEncPosition() >= 0) {
        if ((base + (loc * encoder_cpr)) - GetTurnEncPosition() < -encoder_cpr/2)
            base += encoder_cpr;
        else if ((base + (loc * encoder_cpr)) - GetTurnEncPosition() > encoder_cpr/2)
            base -= encoder_cpr;
        double pos = ((loc * encoder_cpr) + (base));
        turnTalon->Set(TalonFXControlMode::Position, pos);
    } else {
        if ((base - ((1-loc) * encoder_cpr)) - GetTurnEncPosition() < -encoder_cpr/2)
            base += encoder_cpr;
        else if ((base -((1-loc) * encoder_cpr)) - GetTurnEncPosition() > encoder_cpr/2)
            base -= encoder_cpr;
        double pos = (base - (((1-loc) * encoder_cpr)));
        turnTalon->Set(TalonFXControlMode::Position, pos);	
    }
}

double DriveModule::GetDriveEncPosition() {
    return driveTalon->GetSensorCollection().GetIntegratedSensorPosition();
}

double DriveModule::GetTurnEncPosition() {
    return turnTalon->GetSensorCollection().GetIntegratedSensorPosition();
}

void DriveModule::SetIdleMode(IdleMode mode) {
    driveTalon->SetNeutralMode((ctre::phoenix::motorcontrol::NeutralMode) mode);
}

void DriveModule::StopDrive() {
    driveTalon->Set(0);
}

void DriveModule::StopBoth() {
    driveTalon->Set(0);
    turnTalon->Set(0);
}