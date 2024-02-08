package frc.robot.hardware;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.interfaces.Ivert_adjust_HW;

@SuppressWarnings("removal")
public class vert_adjust_HW implements Ivert_adjust_HW {
    private TalonFX motor;
    
    public vert_adjust_HW() {
        motor = new TalonFX(40);//#!RANDOM NUMBER!##>
    }

    public void setPower(double power){
        motor.set(ControlMode.PercentOutput, power);
    }

    public void angleChange(double diff){
        motor.set(ControlMode.PercentOutput, diff);//yet to be implemented
    }

    


}
