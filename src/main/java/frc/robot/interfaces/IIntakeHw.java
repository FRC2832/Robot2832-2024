package frc.robot.interfaces;

public interface IIntakeHw {
    void setIntakePower(double power);
    double getIntakeRpm();
    void setMidPower(double power);
    double getMidRpm();
    boolean isPieceSeen();
    void updateInputs();
}
