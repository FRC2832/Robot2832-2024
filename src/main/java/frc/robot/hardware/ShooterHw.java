package frc.robot.hardware;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.interfaces.IShooterHw;

import org.livoniawarriors.Logger;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

@SuppressWarnings("removal")
public class ShooterHw implements IShooterHw {
    private TalonFX[] shooters;
    TalonFXConfiguration allConfigs = new TalonFXConfiguration();
    private final double UNITS_TO_RPM = (60. * 10.) / 2048.;

    public ShooterHw() {
        shooters = new TalonFX[2];
        shooters[0] = new TalonFX(5, RobotContainer.kCanBusName);
        shooters[1] = new TalonFX(6, RobotContainer.kCanBusName);

        for(TalonFX motor:shooters){
            //motors MUST be reset every powerup!!!
            //motor.configFactoryDefault();
            motor.getAllConfigs(allConfigs);
            allConfigs.slot0.kP = 0.3023;
            allConfigs.slot0.kI = 0.0006;
            allConfigs.slot0.kD = 0;
            allConfigs.slot0.kF = 0.05522;
            allConfigs.slot0.integralZone = 150;
            allConfigs.slot0.allowableClosedloopError = 0;
            allConfigs.motionCruiseVelocity = 0;
            allConfigs.motionAcceleration = 0;
            allConfigs.peakOutputForward = 1;
            allConfigs.peakOutputReverse = -1;
            allConfigs.supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 70, 90, .2);
            motor.configAllSettings(allConfigs);

            motor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
            motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
        }
        Logger.RegisterTalon("Left Shooter",shooters[1]);
        Logger.RegisterTalon("Right Shooter",shooters[0]);


        shooters[1].setInverted(true);
        shooters[0].setInverted(false);
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
