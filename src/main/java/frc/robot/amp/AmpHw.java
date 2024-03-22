package frc.robot.amp;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class AmpHw implements IAmpHw {
    private DoubleSolenoid amp;

    public AmpHw() {
        amp = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 14);
    }

    public void ToggleAmp() {
        if(amp.get() == Value.kOff) {
            amp.set(Value.kForward);
        } else {
            amp.toggle();
        }
    }

    public void SetAmpDirection(boolean up) {
        if(up) {
            amp.set(Value.kForward);
        } else {
            amp.set(Value.kReverse);
        }
    }

    public Value getDirection() {
        return amp.get();
    }
}
