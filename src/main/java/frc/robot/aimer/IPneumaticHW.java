package frc.robot.aimer;

public interface IPneumaticHW {
    void updateInputs();
    double getAngle();
    void driveUp();
    void driveDown();
    void stop();
}