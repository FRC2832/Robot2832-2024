package frc.robot.amp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Amp extends SubsystemBase {
    private IAmpHw hw;

    public Amp(IAmpHw hw) {
        super();
        this.hw = hw;
    }

    public void ToggleAmp() {
        hw.ToggleAmp();
    }

    public void SetAmpDirection(boolean up) {
        hw.SetAmpDirection(up);
    }
}
