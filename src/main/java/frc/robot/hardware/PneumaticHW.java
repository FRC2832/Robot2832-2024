package frc.robot.hardware;

import frc.robot.interfaces.IPneumaticHW;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

@SuppressWarnings("removal")
public class PneumaticHW implements IPneumaticHW {
    private Pigeon2  angSensor;
    private DoubleSolenoid solenoid;
    private double moe = 3.00;
    public PneumaticHW() {
        this.angSensor = new Pigeon2(0);//Change PORT!!!
        this.solenoid = new DoubleSolenoid(0, PneumaticsModuleType.REVPH, 0, 1);
        SmartDashboard.putNumber("Pnuematics Angle", angSensor.getPitch());
        //^^^^^ NOT CONFIGURED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    }

    /**@param target The target angle the pnuematics should go to */
    public void goTo(double target) {
        double currentAngle = angSensor.getPitch();
        if(currentAngle <= (target + moe) && currentAngle >= (target - moe)){
            solenoid.set(kOff);  
        }
        else {
            solenoid.set(currentAngle < target ? kForward : kReverse);
        }
    }

    @Override
    public void updateInputs() {
        SmartDashboard.putNumber("Pnuematics Angle", angSensor.getPitch());
    }
}