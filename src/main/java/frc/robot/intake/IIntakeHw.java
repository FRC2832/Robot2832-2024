package frc.robot.intake;

public interface IIntakeHw {
    void setIntake(boolean isRunning, boolean inverted);
    double getPercentOutput();
    boolean isRunning();
    boolean isInverted();
    boolean isPieceSeen();
    void removeInterrupt();
    void updateInputs();
    void setPower(double power);
    void setRpm(double rpm);
}
