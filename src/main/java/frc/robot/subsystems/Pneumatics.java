package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IPneumaticHW;

public class Pneumatics extends SubsystemBase {
    IPneumaticHW hardware;

    public Pneumatics(IPneumaticHW hardware) {
        super();
        this.hardware = hardware;
    }

    public void goTo(double target) {
        this.hardware.goTo(target);
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
    }
}