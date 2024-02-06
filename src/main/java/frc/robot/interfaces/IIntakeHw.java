package frc.robot.interfaces;

public interface IIntakeHw {
    void setIntake(boolean isRunning, boolean inverted);
    double getPercentOutput();
    boolean isRunning();
    boolean isInverted();
    boolean isPieceSeen();
    void updateInputs();
}
