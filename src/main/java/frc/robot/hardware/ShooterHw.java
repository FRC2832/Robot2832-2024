package frc.robot.hardware;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IShooterHw;
import frc.robot.subsystems.Shooter;

import org.livoniawarriors.Logger;

public class ShooterHw implements IShooterHw {
    private CANSparkFlex[] shooters;
    private SparkPIDController[] pids;
    private RelativeEncoder[] encoders;

    public ShooterHw() {
        shooters = new CANSparkFlex[4];
        pids = new SparkPIDController[4];
        encoders = new RelativeEncoder[4];
        shooters[0] = new CANSparkFlex(1, MotorType.kBrushless);
        shooters[1] = new CANSparkFlex(2, MotorType.kBrushless);
        shooters[2] = new CANSparkFlex(3, MotorType.kBrushless);
        shooters[3] = new CANSparkFlex(4, MotorType.kBrushless);

    
        shooters[1].setInverted(true);
        shooters[3].setInverted(true);

        final double MAX_MOTOR_RPM = 6450;
        for(int i=0; i<shooters.length; i++) {
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
        }
    }

    @Override
    public void setPower(double power) {
        for (CANSparkFlex shooter : shooters) {
            shooter.set(power);
        }
    }

    @Override
    public void setRpm(double rpm) {
        for (int i=0;i<pids.length; i++) {
            pids[i].setReference(rpm, ControlType.kVelocity);
        }
    }
    
    @Override
    public double getCurrentRPM(int shooterID) {        
        return encoders[shooterID].getVelocity();
    }

    /** @param shooterID should be less than length of shooters array */
    public void setIndividualPower(int shooterID, double newPower) {
        CANSparkFlex shooter = shooters[shooterID - 1];
        shooter.set(newPower);
    }

    @Override
    public void updateInputs() {
        SmartDashboard.putNumber("Shooter0 Speed", shooters[0].getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter1 Speed", shooters[1].getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter2 Speed", shooters[2].getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter3 Speed", shooters[3].getEncoder().getVelocity());
    }


}
