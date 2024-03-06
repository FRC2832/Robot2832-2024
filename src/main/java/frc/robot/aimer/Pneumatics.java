package frc.robot.aimer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    IPneumaticHW hardware;
    private double moe = 4.00;

    public Pneumatics(IPneumaticHW hardware) {
        super();
        this.hardware = hardware;
    }

    public void goTo(double target) {
        double currentAngle = hardware.getAngle();
        if(((target - moe) < currentAngle) && (currentAngle < (target + moe))){
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

    public double getAngle() {
        return hardware.getAngle();
    }

    @Override
    public void periodic() {
        hardware.updateInputs();
    }
}