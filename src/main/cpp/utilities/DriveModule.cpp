#include "utilities/DriveModule.h"
#include <frc2/command/PIDCommand.h>
#include <frc2/command/PIDSubsystem.h>

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

    turnTalon->Config_kP(0, 0);
    turnTalon->Config_kI(0, 0);
    turnTalon->Config_kD(0, 0);
    turnTalon->Config_kF(0, 0);

    turnTalon->SetNeutralMode(NeutralMode::Brake);

    turnTalon->SetSensorPhase(true);
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
    double enc_loc = loc / degreePerEncoder;
    double turnp = 0.001;
    double turni = 0;
    double turnd = 0.00000;

    frc2::PIDController pid{turnp, turni, turnd};
    double speed = pid.Calculate(GetTurnEncPosition(), enc_loc);
    speed = std::max(std::min(speed, 0.5), -0.5);
    printf("speed: %f\n", speed);

    SetTurnSpeed(-speed);

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
    return (turnTalon->GetSelectedSensorPosition() * degreePerEncoder);
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