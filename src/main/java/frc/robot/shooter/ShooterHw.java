package frc.robot.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

import org.livoniawarriors.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterHw extends Shooter {
    private TalonFX[] shooters;
    private final double UNITS_TO_RPM = 60;  //rps to rpm is just 60x
    final VelocityVoltage velocityCommand;
    final DutyCycleOut percentCommand;
    final StatusSignal<Double> leftVelocity;
    final StatusSignal<Double> rightVelocity;

    public ShooterHw() {
        shooters = new TalonFX[2];
        shooters[0] = new TalonFX(5, RobotContainer.kCanBusName);  //right
        shooters[1] = new TalonFX(6, RobotContainer.kCanBusName);  //left

        configureMotors();
        shooters[0].setInverted(true);
        shooters[1].setControl(new Follower(shooters[0].getDeviceID(), true));
        Logger.RegisterTalon("Left Shooter",shooters[1]);
        Logger.RegisterTalon("Right Shooter",shooters[0]);

        velocityCommand = new VelocityVoltage(0);
        velocityCommand.Slot = 0;

        percentCommand = new DutyCycleOut(0);
        rightVelocity = shooters[0].getVelocity();
        leftVelocity = shooters[1].getVelocity();
    }

    public void configureMotors() {
        for(TalonFX motor:shooters){
            var slot0Configs = new Slot0Configs();
            slot0Configs.kV = 0.12;
            slot0Configs.kP = 0.52;
            slot0Configs.kI = 0.32;
            slot0Configs.kD = 0;
            motor.getConfigurator().apply(slot0Configs, 0.050);
            /*
            TODO: Implement these motor configs
            allConfigs.slot0.integralZone = 500;
            allConfigs.slot0.allowableClosedloopError = 0;
            allConfigs.motionCruiseVelocity = 0;
            allConfigs.motionAcceleration = 0;
            allConfigs.peakOutputForward = 1;
            allConfigs.peakOutputReverse = -1;
            allConfigs.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 70, 90, .2);
            motor.configAllSettings(allConfigs);
            */
            motor.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    @Override
    protected void setRpmHw(double rpm) {
        shooters[0].setControl(velocityCommand.withVelocity(rpm / UNITS_TO_RPM));
    }

    public void setPower(double power) {
        shooters[0].setControl(percentCommand.withOutput(power));
    }
    
    @Override
    public double getCurrentRPM(int shooterID) {
        StatusSignal<Double> data;        
        if(shooterID == 0) {
            data = leftVelocity;
        } else {
            data = rightVelocity;
        }
        return data.getValueAsDouble() * UNITS_TO_RPM;
    }

    @Override
    public void updateInputs() {
        SmartDashboard.putNumber("Shooter Left Speed", leftVelocity.getValueAsDouble() * UNITS_TO_RPM);
        SmartDashboard.putNumber("Shooter Right Speed", rightVelocity.getValueAsDouble() * UNITS_TO_RPM);
        leftVelocity.refresh();
        rightVelocity.refresh();
    }
}
