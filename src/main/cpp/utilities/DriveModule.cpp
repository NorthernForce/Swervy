#include "utilities/DriveModule.h"

DriveModule::DriveModule(uint8_t driveTalonID, uint8_t turnTalonID, uint8_t canCoderID, double tP, double tI, double tD, int tIZone)
 : canCoderID(canCoderID) {
    driveTalon = std::make_shared<WPI_TalonFX>(driveTalonID);
    turnTalon = std::make_shared<WPI_TalonFX>(turnTalonID);
    canCoder = std::make_shared<CANCoder>(canCoderID);

    ConfigureDriveTalon(driveTalon);
    ConfigureTurnTalon(turnTalon);
}

void DriveModule::ConfigureDriveTalon(std::shared_ptr<WPI_TalonFX> talon, double tP, double tI, double tD) {
    talon->ConfigFactoryDefault();
    talon->SetNeutralMode(NeutralMode::Brake);
    
    /* Config neutral deadband to be the smallest possible */
    talon->ConfigNeutralDeadband(0.001);

    talon->ConfigRemoteFeedbackFilter(canCoderID, RemoteSensorSource::RemoteSensorSource_TalonFX_SelectedSensor, 0, timeoutMs);
    talon->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, pidLoopIdx, timeoutMs);                   

    talon->ConfigNominalOutputForward(0, timeoutMs);
    talon->ConfigNominalOutputReverse(0, timeoutMs);
    talon->ConfigPeakOutputForward(1, timeoutMs);
    talon->ConfigPeakOutputReverse(-1, timeoutMs);

    // talon->Config_kF(pidLoopIdx, ff, timeoutMs);
    talon->Config_kP(pidLoopIdx, tP, timeoutMs);
    talon->Config_kI(pidLoopIdx, tI, timeoutMs);
    talon->Config_kD(pidLoopIdx, tD, timeoutMs);

    talon->ConfigOpenloopRamp(2);
    talon->SetInverted(false);
}

void DriveModule::ConfigureTurnTalon(std::shared_ptr<WPI_TalonFX> talon, double tP, double tI, double tD) {
    turnTalon->ConfigFactoryDefault();
    turnTalon->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0);
    turnTalon->ConfigRemoteFeedbackFilter(canCoderID, RemoteSensorSource::RemoteSensorSource_CANCoder, 0);

    turnTalon->Set(ControlMode::Position, 0);

    turnTalon->Config_kP(0, p);
    turnTalon->Config_kI(0, i);
    turnTalon->Config_kD(0, d);
    turnTalon->Config_kF(0, ff);

    turnTalon->SetNeutralMode(NeutralMode::Brake);

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
    double base = GetTurnEncPosition() * encoder_coeff;
    if (GetTurnEncPosition() >= 0) {
        if ((base + (loc * encoder_coeff)) - GetTurnEncPosition() < -encoder_coeff/2)
            base += encoder_coeff;
        else if ((base + (loc * encoder_coeff)) - GetTurnEncPosition() > encoder_coeff/2)
            base -= encoder_coeff;
        double pos = ((loc * encoder_coeff) + (base));
        //pos = (loc/encoder_coeff);
        printf("pos < 0: %f\n", pos);
        SetTurnPosition(pos);

    } else {
        if ((base - ((1-loc) * encoder_coeff)) - GetTurnEncPosition() < -encoder_coeff/2)
            base += encoder_coeff;
        else if ((base -((1-loc) * encoder_coeff)) - GetTurnEncPosition() > encoder_coeff/2)
            base -= encoder_coeff;
        double pos = (base - ((1-loc) * encoder_coeff));
        //pos = (loc/encoder_coeff);
        printf("pos > 0: %f\n", pos);
        SetTurnPosition(pos);
        
    }
}

double DriveModule::GetDriveEncPosition() {
    return driveTalon->GetSensorCollection().GetIntegratedSensorPosition();
}

double DriveModule::GetTurnEncPosition() {
    // return (turnTalon->GetSelectedSensorPosition() * encoder_coeff);
    return turnTalon->GetSelectedSensorPosition();
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