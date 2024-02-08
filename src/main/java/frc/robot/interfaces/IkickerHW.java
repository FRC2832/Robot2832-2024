package frc.robot.interfaces;

public interface IkickerHW {
    void setPower(double power);
    void setRpm(double rpm);
    void updateInputs();
    double getCurrentRPM(int shooterID);
}
