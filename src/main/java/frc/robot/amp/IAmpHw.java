package frc.robot.amp;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface IAmpHw {
    void ToggleAmp();
    void SetAmpDirection(boolean up);
    Value getDirection();
}
