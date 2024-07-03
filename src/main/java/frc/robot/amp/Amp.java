package frc.robot.amp;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Amp extends SubsystemBase {
    public abstract void ToggleAmp();
    public abstract void SetAmpDirection(boolean up);
    public abstract Value getDirection();

    public Amp() {
        super();
    }

    public boolean IsRaised() {
        return getDirection() == Value.kForward;
    }

    public Command toggleAmp() {
        return runOnce(this::ToggleAmp).withName("AmpToggle");
    }
}
