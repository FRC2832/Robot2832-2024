package frc.robot.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

@SuppressWarnings("removal")
public class ShooterHw implements IShooterHw {
    private TalonFX[] shooters;
    private final double UNITS_TO_RPM = (60. * 10.) / 2048.;

    public ShooterHw() {
        shooters = new TalonFX[2];
        shooters[0] = new TalonFX(5, RobotContainer.kCanBusName);  //right
        shooters[1] = new TalonFX(6, RobotContainer.kCanBusName);  //left

        configureMotors();
        
        for(TalonFX motor:shooters){
            motor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
            motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
        }
        Logger.RegisterTalon("Left Shooter",shooters[1]);
        Logger.RegisterTalon("Right Shooter",shooters[0]);
    }

    public void configureMotors() {
        TalonFXConfiguration allConfigs = new TalonFXConfiguration();
        for(TalonFX motor:shooters){
            motor.getAllConfigs(allConfigs);
            allConfigs.slot0.kP = 0.4023;
            allConfigs.slot0.kI = 0.0016;
            allConfigs.slot0.kD = 0.01;
            allConfigs.slot0.kF = 0.05522;
            allConfigs.slot0.integralZone = 500;
            allConfigs.slot0.allowableClosedloopError = 0;
            allConfigs.motionCruiseVelocity = 0;
            allConfigs.motionAcceleration = 0;
            allConfigs.peakOutputForward = 1;
            allConfigs.peakOutputReverse = -1;
            allConfigs.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 70, 90, .2);
            motor.configAllSettings(allConfigs);
        }

        shooters[1].setInverted(false);
        shooters[0].setInverted(true);
    }

    @Override
    public void setRpm(double rpm) {
        for (TalonFX shooter : shooters) {
            shooter.set(TalonFXControlMode.Velocity, rpm / UNITS_TO_RPM);
        }
    }

    public void setPower(double power) {
        for (TalonFX shooter : shooters) {
            shooter.set(TalonFXControlMode.PercentOutput, power);
        }
    }
    
    @Override
    public double getCurrentRPM(int shooterID) {        
        return shooters[shooterID].getSelectedSensorVelocity() * UNITS_TO_RPM;
    }

    /** @param shooterID should be less than length of shooters array */
    public void setIndividualPower(int shooterID, double newPower) {
        TalonFX shooter = shooters[shooterID - 1];
        shooter.set(TalonFXControlMode.PercentOutput, newPower);
    }

    @Override
    public void updateInputs() {
        SmartDashboard.putNumber("Shooter0 Speed", shooters[0].getSelectedSensorVelocity() * UNITS_TO_RPM);
        SmartDashboard.putNumber("Shooter1 Speed", shooters[1].getSelectedSensorVelocity() * UNITS_TO_RPM);
    }
}
