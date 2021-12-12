#include <ctre/Phoenix.h>
#include <memory>

class DriveModule {
 public:
    enum IdleMode {
        Coast = ctre::phoenix::motorcontrol::NeutralMode::Coast,
        Brake = ctre::phoenix::motorcontrol::NeutralMode::Brake       
    };

    DriveModule(uint8_t driveTalonID, uint8_t turnTalonID, double tP, double tI, double tD, int tIZone);
    void ConfigureTalon(std::shared_ptr<WPI_TalonFX> talon);
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
    const uint16_t encoder_cpr = 4096;
};