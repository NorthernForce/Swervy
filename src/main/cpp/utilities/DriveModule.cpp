#include "utilities/DriveModule.h"

DriveModule::DriveModule(uint8_t driveTalonID, uint8_t turnTalonID, uint8_t canCoderID, double tP, double tI, double tD, int tIZone)
 : canCoderID(canCoderID) {
    driveTalon = std::make_shared<WPI_TalonFX>(driveTalonID);
    turnTalon = std::make_shared<WPI_TalonFX>(turnTalonID);
    canCoder = std::make_shared<CANCoder>(canCoderID);

    ConfigureDriveTalon();
    ConfigureTurnTalon();
}

void DriveModule::ConfigureDriveTalon() {
    driveTalon->ConfigFactoryDefault();
    driveTalon->SetNeutralMode(NeutralMode::Brake);
    
    /* Config neutral deadband to be the smallest possible */
    driveTalon->ConfigNeutralDeadband(0.001);

    driveTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);

    driveTalon->ConfigNominalOutputForward(0, timeoutMs);
    driveTalon->ConfigNominalOutputReverse(0, timeoutMs);
    driveTalon->ConfigPeakOutputForward(1, timeoutMs);
    driveTalon->ConfigPeakOutputReverse(-1, timeoutMs);

    driveTalon->Config_kP(pidLoopIdx, 1, timeoutMs);
    driveTalon->Config_kI(pidLoopIdx, 0, timeoutMs);
    driveTalon->Config_kD(pidLoopIdx, 0, timeoutMs);

    driveTalon->ConfigOpenloopRamp(2);
    driveTalon->SetInverted(false);
}

void DriveModule::ConfigureTurnTalon() {
    turnTalon->ConfigFactoryDefault();
    turnTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0);
    turnTalon->ConfigRemoteFeedbackFilter(canCoderID, RemoteSensorSource::RemoteSensorSource_CANCoder, 0);

    turnTalon->Config_kP(1, 0);
    turnTalon->Config_kI(0, 0);
    turnTalon->Config_kD(0, 0);
    turnTalon->Config_kF(ff, 0);

    turnTalon->SetNeutralMode(NeutralMode::Brake);
    turnTalon->ConfigOpenloopRamp(2);

    turnTalon->SetSensorPhase(false);
}

void DriveModule::SetDriveSpeed(double speed) {
    driveTalon->Set(speed);
}

void DriveModule::SetTurnSpeed(double speed) {
    turnTalon->Set(speed);
}

void DriveModule::SetTurnPosition(double pos) {
    turnTalon->Set(TalonFXControlMode::Position, pos);
}

void DriveModule::SetTurnLocation(double loc) {
    printf("Loc: %f\n", loc);
    double enc_loc = loc - offsets[canCoderID-firstEncoderID];
    
    //printf("encoder pos: %f, location: %f\n", GetTurnEncPosition(), enc_loc);
    double speed = pid.Calculate(GetTurnEncPosition(), loc);
    speed = std::max(std::min(speed, 0.5), -0.5);
    //printf("speed: %f\n", speed);

    SetTurnSpeed(speed);

//*******************************************************

    // int encoder_cpr = 4096;
    // double base = GetTurnEncPosition() * encoder_cpr;
    // if (GetTurnEncPosition() >= 0) {
    //     if ((base + (loc * encoder_cpr)) - GetTurnEncPosition() < -encoder_cpr/2)
    //         base += encoder_cpr;
    //     else if ((base + (loc * encoder_cpr)) - GetTurnEncPosition() > encoder_cpr/2)
    //         base -= encoder_cpr;
    //     double pos = ((loc * encoder_cpr) + (base));
    //     //printf("pos < 0: %f", pos);
    //     turnTalon->Set(TalonFXControlMode::Position, pos);
    // } else {
    //     if ((base - ((1-loc) * encoder_cpr)) - GetTurnEncPosition() < -encoder_cpr/2)
    //         base += encoder_cpr;
    //     else if ((base -((1-loc) * encoder_cpr)) - GetTurnEncPosition() > encoder_cpr/2)
    //         base -= encoder_cpr;
    //     double pos = (base - (((1-loc) * encoder_cpr)));
    //     turnTalon->Set(TalonFXControlMode::Position, pos);
        //printf("pos > 0: %f", pos);
    // }
}

double DriveModule::GetDriveEncPosition() {
    return driveTalon->GetSensorCollection().GetIntegratedSensorPosition();
}

double DriveModule::GetTurnEncPosition() {
    return (turnTalon->GetSelectedSensorPosition() * degreePerEncoder) - offsets[canCoderID-firstEncoderID];
}

void DriveModule::ResetEncoder() {
    turnTalon->SetSelectedSensorPosition(0);
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