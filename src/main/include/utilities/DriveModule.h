#include <ctre/Phoenix.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/PIDSubsystem.h>
#include <memory>
#include <vector>
#include "Constants.h"

class DriveModule {
 public:
    enum IdleMode {
        Coast = ctre::phoenix::motorcontrol::NeutralMode::Coast,
        Brake = ctre::phoenix::motorcontrol::NeutralMode::Brake       
    };

    DriveModule(uint8_t driveTalonID, uint8_t turnTalonID, uint8_t canCoderID, double tP, double tI, double tD, int tIZone);
    void ConfigureDriveTalon();
    void ConfigureTurnTalon();
    void SetDriveSpeed(double speed);
    void SetTurnSpeed(double speed);
    void SetTurnPosition(double pos);
    void SetTurnLocation(double loc);
    double GetTurnEncPosition();
    double GetDriveEncPosition();
    void ResetEncoder();
    void SetIdleMode(IdleMode mode);
    void StopDrive();
    void StopBoth();

 private:
    std::shared_ptr<WPI_TalonFX> driveTalon;
    std::shared_ptr<WPI_TalonFX> turnTalon;
    std::shared_ptr<CANCoder> canCoder;
    const double degreePerEncoder = (360.0/4096.0);

    double p = 0.5;
    double i = 0;
    double d = 0; 
    double ff = (1023/20660);
    const uint16_t maxI = 300;
    const double maxOutput = 1;
    const double minOutput = -1;
    uint8_t timeoutMs = 30;
    uint8_t pidLoopIdx = 0; //default

    uint8_t canCoderID;

    // turn PID
    double turnp = 0.002;
    double turni = 0.0;
    double turnd = 0.00000;

    frc2::PIDController pid{turnp, turni, turnd};
    double offsets[4] = {
        256.289,
        136.033,
        4.572,
        293.203
    };
    uint8_t firstEncoderID = Constants::EncoderIDs::encoderFL;
};