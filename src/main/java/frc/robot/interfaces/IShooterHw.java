package frc.robot.interfaces;

public interface IShooterHw {
    void setPower(double power);
    void setRpm(double rpm);
    void setIndividualPower(int shooterID, double newPower);
    void updateInputs();
}
