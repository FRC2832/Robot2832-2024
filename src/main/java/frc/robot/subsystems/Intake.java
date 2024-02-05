package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    }

    public boolean isPieceDetected() {
        return hardware.isPieceSeen();
    }

    public void runMotor() {
        
    }

    /**
     * @param power Percent -1 to 1
     */
    public void setPower(double power) {

    }
}
