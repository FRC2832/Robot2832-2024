package frc.robot.kicker;

public interface IKickerHw {
    void setPower(double power);
    void setRpm(double rpm);
    void updateInputs();
    double getCurrentRPM(int shooterID);
}
