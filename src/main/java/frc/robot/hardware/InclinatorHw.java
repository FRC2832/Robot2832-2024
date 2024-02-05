package frc.robot.hardware;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.interfaces.IInclinatorHw;

@SuppressWarnings("removal")
public class InclinatorHw implements IInclinatorHw {
    private TalonFX leftClimb;
    private TalonFX rightClimb;
    
    public InclinatorHw() {
        leftClimb = new TalonFX(60);
        rightClimb = new TalonFX(61);
    }

    public void setPower(double power){
        leftClimb.set(ControlMode.PercentOutput, power);
        rightClimb.set(ControlMode.PercentOutput, power);
    }

    public void setPower(double powerLeft, double powerRight){
        leftClimb.set(ControlMode.PercentOutput, powerLeft);
        rightClimb.set(ControlMode.PercentOutput, powerRight);
    }

}
