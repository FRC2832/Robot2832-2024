package frc.robot.hardware;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IKickerHw;
import org.livoniawarriors.Logger;

public class KickerHw implements IKickerHw {
//
    private CANSparkFlex[] kickers;
    private SparkPIDController[] pids;
    private RelativeEncoder[] encoders;

    public KickerHw() {
        kickers = new CANSparkFlex[2];
        pids = new SparkPIDController[2];
        encoders = new RelativeEncoder[2];
        kickers[0] = new CANSparkFlex(3, MotorType.kBrushless);
        kickers[1] = new CANSparkFlex(4, MotorType.kBrushless);
    
        kickers[1].setInverted(true);
        kickers[0].setInverted(false);

        final double MAX_MOTOR_RPM = 6450;
        for(int i=0; i<kickers.length; i++) {
            pids[i] = kickers[i].getPIDController();
            pids[i].setP(1./MAX_MOTOR_RPM);
            pids[i].setI(0.01/MAX_MOTOR_RPM);
            pids[i].setD(0);
            pids[i].setFF(1./MAX_MOTOR_RPM);
            pids[i].setIZone(100);

            encoders[i] = kickers[i].getEncoder();
            final int wheel = i;
            Logger.RegisterSensor("Kicker " + i + " RPM", () -> getCurrentRPM(wheel));
            //Logger.RegisterCanSparkMax(shooters[wheel]);
        }
    }
    @Override
    public double getCurrentRPM(int shooterID) {        
        return encoders[shooterID].getVelocity();
    }

    @Override
    public void setPower(double power) {
        for (CANSparkFlex kicker : kickers) {
            kicker.set(power);
        }
    }
    @Override
    public void setRpm(double rpm) {
        for (int i=0;i<pids.length; i++) {
            pids[i].setReference(rpm, ControlType.kVelocity);
        }
    }
    


    @Override
    public void updateInputs() {
        SmartDashboard.putNumber("Kicker0 Speed", kickers[0].getEncoder().getVelocity());
        SmartDashboard.putNumber("Kicker1 Speed", kickers[1].getEncoder().getVelocity());
    }


}
