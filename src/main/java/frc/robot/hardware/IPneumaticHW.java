package frc.robot.hardware;

import frc.robot.interfaces.IPneumaticHW;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


@SuppressWarnings("removal")
public class PneumaticHW implements IPneumaticHW {
    
    private Pigeon2  angSensor;
    private DoubleSolenoid solenoid;
    public PneumaticHW() {
        this.angSensor = new Pigeon2(0);//Change PORT!!!
        this.solenoid = new DoubleSolenoid(0, PneumaticsModuleType.REVPH, 0, 0);
        //^^^^^ NOT CONFIGURED

    }

    public void stop(double target){
        if (angSensor.getPitch()==target){
            solenoid.set(kOff);  
        }
    }
    public void start(boolean up){

        if(up){solenoid.set(kForward);}else{solenoid.set(kReverse);}
    }

}