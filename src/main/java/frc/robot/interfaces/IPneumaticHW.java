package frc.robot.interfaces;

public interface IPneumaticHW {
    void updateInputs();
    double getAngle();
    void driveUp();
    void driveDown();
    void stop();
}