package frc.robot.hardware;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.interfaces.IVertAdjustHw;

@SuppressWarnings("removal")
public class VertAdjustHw implements IVertAdjustHw {
    private TalonFX motor;
    
    public VertAdjustHw() {
        motor = new TalonFX(40);//#!RANDOM NUMBER!##>
    }

    public void setPower(double power){
        motor.set(ControlMode.PercentOutput, power);
    }

    public void angleChange(double diff){
        motor.set(ControlMode.PercentOutput, diff);//yet to be implemented
    }

    


}
