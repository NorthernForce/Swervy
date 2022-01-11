#include "utilities/DriveModule.h"

DriveModule::DriveModule(uint8_t driveTalonID, uint8_t turnTalonID, uint8_t canCoderID, double tP, double tI, double tD, int tIZone) {
    driveTalon = std::make_shared<WPI_TalonFX>(driveTalonID);
    turnTalon = std::make_shared<WPI_TalonFX>(turnTalonID);

    ConfigureTalon(driveTalon);
    ConfigureTalon(turnTalon);
}

void DriveModule::ConfigureTalon(std::shared_ptr<WPI_TalonFX> talon, double tP, double tI, double tD) {
    talon->ConfigFactoryDefault();
    talon->SetNeutralMode(NeutralMode::Brake);
    
    /* Config neutral deadband to be the smallest possible */
    talon->ConfigNeutralDeadband(0.001);

    talon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, pidLoopIdx, timeoutMs);
                                        
    talon->ConfigNominalOutputForward(0, timeoutMs);
    talon->ConfigNominalOutputReverse(0, timeoutMs);
    talon->ConfigPeakOutputForward(1, timeoutMs);
    talon->ConfigPeakOutputReverse(-1, timeoutMs);

    talon->Config_kF(pidLoopIdx, ff, timeoutMs);
    talon->Config_kP(pidLoopIdx, tP, timeoutMs);
    talon->Config_kI(pidLoopIdx, tI, timeoutMs);
    talon->Config_kD(pidLoopIdx, tD, timeoutMs);

    talon->ConfigOpenloopRamp(2);
    talon->SetInverted(false);
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
        //printf("pos < 0: %f", pos);
        turnTalon->Set(TalonFXControlMode::Position, pos);
    } else {
        if ((base - ((1-loc) * encoder_cpr)) - GetTurnEncPosition() < -encoder_cpr/2)
            base += encoder_cpr;
        else if ((base -((1-loc) * encoder_cpr)) - GetTurnEncPosition() > encoder_cpr/2)
            base -= encoder_cpr;
        double pos = (base - (((1-loc) * encoder_cpr)));
        turnTalon->Set(TalonFXControlMode::Position, pos);
        //printf("pos > 0: %f", pos);
    }
}

double DriveModule::GetDriveEncPosition() {
    return driveTalon->GetSensorCollection().GetIntegratedSensorPosition();
}

double DriveModule::GetTurnEncPosition() {
    return turnTalon->GetSensorCollection().GetIntegratedSensorPosition() / 34.55;
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