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
    public void diagnosticCheck(){
        String[] results = new String[]{"false", "false", "false"};
        
        goTo(30); 
        if(hardware.getAngle() <= (30 + moe) && hardware.getAngle() >= (30 - moe)){
            results[0] = "true";
        }
        goTo(40);
        if(hardware.getAngle() <= (40 + moe) && hardware.getAngle() >= (40 - moe)){
            results[1]="true";
        }
        goTo(50);
        if(hardware.getAngle() <= (50 + moe) && hardware.getAngle() >= (50 - moe)){
            results[2]="true";
        }
        
        SmartDashboard.putString("Angle Check 30°", results[0] );
        SmartDashboard.putString("Angle Check 40°", results[1] );
        SmartDashboard.putString("Angle Check 50°", results[2] );

    }
    @Override
    public void periodic() {
        hardware.updateInputs();
    }
}