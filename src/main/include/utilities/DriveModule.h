#include <ctre/Phoenix.h>
#include <memory>

class DriveModule {
 public:
    enum IdleMode {
        Coast = ctre::phoenix::motorcontrol::NeutralMode::Coast,
        Brake = ctre::phoenix::motorcontrol::NeutralMode::Brake       
    };

    DriveModule(uint8_t driveTalonID, uint8_t turnTalonID, uint8_t canCoderID, double tP, double tI, double tD, int tIZone);
    void ConfigureTalon(std::shared_ptr<WPI_TalonFX> talon, double tP=1, double tI=0, double tD=0);
    void SetDriveSpeed(double speed);
    void SetTurnSpeed(double speed);
    void SetTurnLocation(double loc);
    double GetTurnEncPosition();
    double GetDriveEncPosition();
    void SetIdleMode(IdleMode mode);
    void StopDrive();
    void StopBoth();

 private:
    std::shared_ptr<WPI_TalonFX> driveTalon;
    std::shared_ptr<WPI_TalonFX> turnTalon;
    const uint16_t encoder_cpr = 2048;

    double p = 0.1;
    double i = 0.001;
    double d = 5; 
    double ff = (1023/20660);
    const uint16_t maxI = 300;
    const double maxOutput = 1;
    const double minOutput = -1;
    uint8_t timeoutMs = 30;
    uint8_t pidLoopIdx = 0; //default
};