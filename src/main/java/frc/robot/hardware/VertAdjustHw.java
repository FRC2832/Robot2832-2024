package frc.robot.hardware;
import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.interfaces.IVertAdjustHw;

@SuppressWarnings("removal")
public class VertAdjustHw implements IVertAdjustHw {
    private TalonFX motor;
    TalonFXConfiguration allConfigs = new TalonFXConfiguration();
    public VertAdjustHw() {
        motor = new TalonFX(40);//#!RANDOM NUMBER!##>
        motor.getAllConfigs(allConfigs);
        allConfigs.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 70, 90, .2);
        Logger.RegisterTalon( "Vertical Adjuster", motor);
    }

    public void setPower(double power){
        motor.set(ControlMode.PercentOutput, power);
    }

    public void angleChange(double diff){
        motor.set(ControlMode.PercentOutput, diff);//yet to be implemented
    }

    


}
