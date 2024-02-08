package frc.robot.hardware;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IShooterHw;
import frc.robot.subsystems.Shooter;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;

import org.livoniawarriors.Logger;
@SuppressWarnings("removal")
public class ShooterHw implements IShooterHw {
    private TalonFX[] shooters;
//    private SparkPIDController[] pids;
    private CANcoder[] encoders;
    TalonFXConfiguration allConfigs = new TalonFXConfiguration();
    public ShooterHw() {
        shooters = new TalonFX[4];
//        pids = new SparkPIDController[4];
//        encoders = new RelativeEncoder[4];
        for(TalonFX motor:shooters){
        //SET ALL VALUES TO 0 FOR NOW
        motor = new TalonFX(1);
        motor.getAllConfigs(allConfigs);
        allConfigs.slot0.kP = 0;
        allConfigs.slot0.kI = 0;
        allConfigs.slot0.kD = 0;
        allConfigs.slot0.kF = 0;
        allConfigs.slot0.integralZone = 0;
        allConfigs.slot0.allowableClosedloopError = 0;
        allConfigs.motionCruiseVelocity = 0;
        allConfigs.motionAcceleration = 0;
        motor.configAllSettings(allConfigs);}

    
        shooters[1].setInverted(true);
        shooters[0].setInverted(false);
//        shooters[3].setInverted(true);

        final double MAX_MOTOR_RPM = 6450;
/*         for(int i=0; i<shooters.length; i++) {
           pids[i] = shooters[i].getPIDController();
            pids[i].setP(1./MAX_MOTOR_RPM);
            pids[i].setI(0.01/MAX_MOTOR_RPM);
            pids[i].setD(0);
            pids[i].setFF(1./MAX_MOTOR_RPM);
            pids[i].setIZone(100);
            encoders[i] = shooters[i].getEncoder();
            final int wheel = i;
            Logger.RegisterSensor("Shooter " + i + " RPM", () -> getCurrentRPM(wheel));
            Logger.RegisterSensor("Shooter " + i + " Velocity", () -> Shooter.RPMToVelocity(getCurrentRPM(wheel)));
            //Logger.RegisterCanSparkMax(shooters[wheel]);
        }*/
    }

    @Override
    public void setRpm(double rpm) {
        for (TalonFX shooter : shooters) {
            shooter.set(TalonFXControlMode.Velocity, rpm);
        }
    }

    public void setPower(double power) {
        for (TalonFX shooter : shooters) {
            shooter.set(TalonFXControlMode.Velocity, 6450*power);//Probably not best method
        }
    }
    //@Override
    // public void setRpm(double rpm) {
    //     for (int i=0;i<pids.length; i++) {
    //         shooter.set(TalonFXControlMode., 100);
    //     }
    //}
    
    @Override
    public double getCurrentRPM(int shooterID) {        
        return shooters[0].getSelectedSensorVelocity();//not sure about this one
    }

    /** @param shooterID should be less than length of shooters array */
    public void setIndividualPower(int shooterID, double newPower) {
        TalonFX shooter = shooters[shooterID - 1];
        shooter.set(TalonFXControlMode.Velocity, 100);
    }

    @Override
    public void updateInputs() {
        SmartDashboard.putNumber("Shooter0 Speed", shooters[0].getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter1 Speed", shooters[1].getSelectedSensorVelocity());
//        SmartDashboard.putNumber("Shooter2 Speed", shooters[2].getEncoder().getVelocity());
//        SmartDashboard.putNumber("Shooter3 Speed", shooters[3].getEncoder().getVelocity());
    }


}
