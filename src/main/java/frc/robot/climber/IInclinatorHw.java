package frc.robot.climber;

public interface IInclinatorHw {
    void setPower(double power);
    void setPower(double powerLeft, double powerRight);
    double getLeftCurrent();
    double getRightCurrent();
    boolean getLeftLimit();
    boolean getRightLimit();
    double getLeftHeight();
    double getRightHeight();
    void updateInputs();
}
