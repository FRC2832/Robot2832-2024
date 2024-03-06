package frc.robot.aimer;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import org.livoniawarriors.Logger;

@SuppressWarnings("removal")
public class PneumaticHW implements IPneumaticHW {
    private Pigeon2  angSensor;
    private DoubleSolenoid solenoid;
    private double currentAngle;
    
    public PneumaticHW() {
        this.angSensor = new Pigeon2(10);
        this.solenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 15, 0);
        
        Logger.RegisterSensor("Shooter Angle", ()->currentAngle);
    }

    @Override
    public void updateInputs() {
        currentAngle = -angSensor.getRoll();
    }

    @Override 
    public double getAngle() {
        return currentAngle;
    }

    @Override
    public void driveUp() {
        solenoid.set(kForward);
    }

    @Override
    public void driveDown() {
        solenoid.set(kReverse);
    }

    @Override
    public void stop() {
        solenoid.set(kOff);
    }
}
