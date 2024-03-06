package frc.robot.hardware;
import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.interfaces.IInclinatorHw;

@SuppressWarnings("removal")
public class InclinatorHw implements IInclinatorHw {
    private TalonFX leftClimb;
    private TalonFX rightClimb;
    private AnalogInput leftSwitch;
    private AnalogInput rightSwitch;
    private TalonFXConfiguration allConfigsL = new TalonFXConfiguration();
    private TalonFXConfiguration allConfigsR = new TalonFXConfiguration();
    private boolean leftLimit;
    private boolean rightLimit;
    
    public InclinatorHw() {
        leftClimb = new TalonFX(62);
        rightClimb = new TalonFX(61);
        leftSwitch = new AnalogInput(1);
        rightSwitch = new AnalogInput(2);

        //motors MUST be reset every powerup!!!
        leftClimb.configFactoryDefault();
        rightClimb.configFactoryDefault();

        //Add Logger Data for Faults 
        leftClimb.getAllConfigs(allConfigsL);
        rightClimb.getAllConfigs(allConfigsR);

        allConfigsL.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 70, 90, .2);
        allConfigsR.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 70, 90, .2);
        allConfigsL.peakOutputForward = 1;
        allConfigsL.peakOutputReverse = -1;
        allConfigsR.peakOutputForward = 1;
        allConfigsR.peakOutputReverse = -1;
        leftClimb.configAllSettings(allConfigsL);
        rightClimb.configAllSettings(allConfigsR);

        leftClimb.setNeutralMode(NeutralMode.Brake);
        rightClimb.setNeutralMode(NeutralMode.Brake);

        Logger.RegisterTalon( "Left Climb", leftClimb);
        Logger.RegisterTalon( "Right Climb", rightClimb);
        Logger.RegisterSensor("Left Climb Stop", () -> leftSwitch.getVoltage());
        Logger.RegisterSensor("Right Climb Stop", () -> rightSwitch.getVoltage());

        leftClimb.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
        leftClimb.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

        rightClimb.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
        rightClimb.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

        leftLimit = false;
        rightLimit = false;
    }

    public void setPower(double power){
        setPower(power, power);
    }

    public void setPower(double powerLeft, double powerRight){
        leftClimb.set(ControlMode.PercentOutput, powerLeft);
        rightClimb.set(ControlMode.PercentOutput, powerRight);

        if(powerLeft > 0.1) {
            leftLimit = false;
            leftClimb.set(ControlMode.PercentOutput, powerLeft);
        } else if (leftLimit) {
            leftClimb.set(ControlMode.PercentOutput, 0);
        } else {
            leftClimb.set(ControlMode.PercentOutput, powerLeft);
        }

        if(powerRight > 0.1) {
            rightLimit = false;
            rightClimb.set(ControlMode.PercentOutput, powerRight);
        } else if (rightLimit) {
            rightClimb.set(ControlMode.PercentOutput, 0);
        } else {
            rightClimb.set(ControlMode.PercentOutput, powerRight);
        }
    }

    public double getLeftCurrent(){
        return leftClimb.getSupplyCurrent();
    }

    public double getRightCurrent(){
        return rightClimb.getSupplyCurrent();
    }

    public boolean getLeftLimit() {
        return leftLimit;
    }

    public boolean getRightLimit() {
        return rightLimit;
    }

    @Override
    public void updateInputs() {
        if (leftSwitch.getVoltage() < 0.1) {
            leftLimit = true;
        }

        if(rightSwitch.getVoltage() < 0.1) {
            rightLimit = true;
        }
    }
}
