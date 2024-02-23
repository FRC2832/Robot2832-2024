package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IIntakeHw;

public class Intake extends SubsystemBase {
    private IIntakeHw hardware;

    public Intake(IIntakeHw hardware) {
        super();
        this.hardware = hardware;
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
        SmartDashboard.putNumber("Intake Percent", getPercentOutput());
    }

    public boolean isPieceDetected() {
        return hardware.isPieceSeen();
    }

    /** @param inverted true = spin backwards, false = spin forwards */
    public void setIntake(boolean isRunning, boolean inverted) {
        hardware.setIntake(isRunning, inverted);
    }

    public double getPercentOutput() {
        return hardware.getPercentOutput();
    }

    public void removeInterrupt() {
        hardware.removeInterrupt();
    }

    public void setPower(double power) {
        hardware.setPower(power);
    }

    public void setRpm(double rpm) {
        hardware.setRpm(rpm);
    }
}
