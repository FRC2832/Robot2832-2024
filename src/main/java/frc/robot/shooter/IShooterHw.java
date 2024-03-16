package frc.robot.shooter;

public interface IShooterHw {
    void setPower(double power);
    void setRpm(double rpm);
    double getCurrentRPM(int shooterID);
    void updateInputs();
}



