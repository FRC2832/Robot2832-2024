package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public void diagnosticCheck() {
        boolean[] results = new boolean[]{false, false, false};
        for(int i = 0; i < 3 ; i++) {
            double testVal = (10 * i) + 30;
            goTo(testVal); 
            if(hardware.getAngle() <= (testVal + moe) && hardware.getAngle() >= (testVal - moe)){
                results[i] = true;
            }
        }
        
        SmartDashboard.putBooleanArray("Angle Check 30°, 40°, 50°", results);
    }
    @Override
    public void periodic() {
        hardware.updateInputs();
    }
}