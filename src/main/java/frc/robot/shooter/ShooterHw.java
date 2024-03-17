package frc.robot.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.livoniawarriors.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterHw implements IShooterHw {
    private TalonFX[] shooters;
    private TalonFX driveShooter;
    private TalonFX followShooter;
    private VelocityVoltage pidRequest;
    private StatusSignal<Double> rpsSpeed;

    public ShooterHw() {
        driveShooter = new TalonFX(5);  //right
        followShooter = new TalonFX(6); //left
        shooters = new TalonFX[2];
        shooters[0] = driveShooter;
        shooters[1]  = followShooter;

        configureMotors();
        
        followShooter.setControl(new Follower(driveShooter.getDeviceID(), true));
        pidRequest = new VelocityVoltage(0).withSlot(0);
        rpsSpeed = driveShooter.getVelocity();

        Logger.RegisterTalon("Left Shooter",shooters[1]);
        Logger.RegisterTalon("Right Shooter",shooters[0]);
    }

    public void configureMotors() {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        configs.SupplyCurrentLimitEnable = true;
        configs.SupplyCurrentLimit = 50;
        configs.SupplyCurrentThreshold = 70;
        configs.SupplyTimeThreshold = 0.2;

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();

        slot0Configs.kS = 0.0; // no output overcome static friction
        slot0Configs.kV = 24./100.; // 6000 RPM over 12V
        slot0Configs.kP = 0.11; // An error of 1 rpm results in 0.011 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        for(TalonFX motor:shooters){
            motor.getConfigurator().apply(configs);
            motor.getConfigurator().apply(slot0Configs);
            motor.setNeutralMode(NeutralModeValue.Coast);
        }

        shooters[1].setInverted(false);
        shooters[0].setInverted(true);
    }

    @Override
    public void setRpm(double rpm) {
        //need to conver rotations per minute to second
        SmartDashboard.putNumber("Shooter Request", rpm);
        driveShooter.setControl(pidRequest.withVelocity(rpm/60));
        //setPower(rpm/6000);
    }

    public void setPower(double power) {
        driveShooter.set(power);
    }
    
    @Override
    public double getCurrentRPM(int shooterID) {     
        rpsSpeed.refresh();   
        return rpsSpeed.getValue();
    }

    @Override
    public void updateInputs() {
        rpsSpeed.refresh(); 
        SmartDashboard.putNumber("Shooter Speed", rpsSpeed.getValue() * 60);
    }
}
