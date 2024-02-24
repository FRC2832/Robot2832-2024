package frc.robot.hardware;
import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.interfaces.IInclinatorHw;

@SuppressWarnings("removal")
public class InclinatorHw implements IInclinatorHw {
    private TalonFX leftClimb;
    private TalonFX rightClimb;
    private TalonFXConfiguration allConfigsL = new TalonFXConfiguration();
    private TalonFXConfiguration allConfigsR = new TalonFXConfiguration();
    
    public InclinatorHw() {
        leftClimb = new TalonFX(61);
        rightClimb = new TalonFX(62);
        //Add Logger Data for Faults 
        leftClimb.getAllConfigs(allConfigsL);
        rightClimb.getAllConfigs(allConfigsR);

        allConfigsL.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 70, 90, .2);
        allConfigsR.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 70, 90, .2);

        leftClimb.configAllSettings(allConfigsL);
        rightClimb.configAllSettings(allConfigsR);

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

    public double getLeftCurrent(){
        return leftClimb.getSupplyCurrent();
    }

    public double getRightCurrent(){
        return rightClimb.getSupplyCurrent();
    }

    @Override
    public void updateInputs() {
    }

}
