package frc.robot.interfaces;

public interface IInclinatorHw {
    void setPower(double power);
    void setPower(double powerLeft, double powerRight);
    void updateInputs();
}