package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IPneumaticHW;

public class Pneumatics extends SubsystemBase {
    IPneumaticHW hardware;
    private double moe = 3.00;

    public Pneumatics(IPneumaticHW hardware) {
        super();
        this.hardware = hardware;
    }

    public void goTo(double target) {
        double currentAngle = hardware.getAngle();
        if(currentAngle <= (target + moe) && currentAngle >= (target - moe)){
            hardware.stop(); 
        }
        else {
            if(currentAngle < target) {
                hardware.driveUp(); 
            } else {
                hardware.driveDown();
            }
        }
    }

    public void driveUp() {
        hardware.driveUp();
    }

    public void driveDown() {
        hardware.driveDown();
    }

    public void stop() {
        hardware.stop();
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
    }
}