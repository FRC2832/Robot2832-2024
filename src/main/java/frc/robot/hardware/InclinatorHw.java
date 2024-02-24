package frc.robot.hardware;
import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.interfaces.IInclinatorHw;

@SuppressWarnings("removal")
public class InclinatorHw implements IInclinatorHw {
    private TalonFX leftClimb;
    private TalonFX rightClimb;
    
    public InclinatorHw() {
        leftClimb = new TalonFX(61);
        rightClimb = new TalonFX(62);
        //Add Logger Data for Faults 
        Logger.RegisterTalon( "Left Climb", leftClimb);
        Logger.RegisterTalon( "Right Climb", rightClimb);
    }

    public void setPower(double power){
        leftClimb.set(ControlMode.PercentOutput, power);
        rightClimb.set(ControlMode.PercentOutput, power);
    }

    public void setPower(double powerLeft, double powerRight){
        leftClimb.set(ControlMode.PercentOutput, powerLeft);
        rightClimb.set(ControlMode.PercentOutput, powerRight);
    }

    @Override
    public void updateInputs() {
    }

}
