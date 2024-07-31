package frc.robot.intake;

import org.livoniawarriors.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeHw extends Intake {
    private TalonFX rightIntake;
    private TalonFX leftIntake;
    private DigitalInput enterSensor;
    private DigitalInput highSensor;
    private boolean isRunning;
    private boolean inverted;
    private boolean interrupt;
    private VelocityVoltage pidRequest;
    private double ratio = 32./50.;

    public IntakeHw() {
        this.rightIntake = new TalonFX(50); //WITH RATIO
        this.leftIntake = new TalonFX(51); //WITH RATIO
        this.enterSensor = new DigitalInput(0);
        this.highSensor = new DigitalInput(1);
        this.isRunning = false;
        this.inverted = false;
        this.interrupt = false;
        //configureMotors();
        
        // create a velocity closed-loop request, voltage output, slot 0 configs
        pidRequest = new VelocityVoltage(0).withSlot(0);

        Logger.RegisterSensor("Low Note", () -> enterSensor.get() ? 0 : 1);
        Logger.RegisterSensor("High Note", () -> highSensor.get() ? 0 : 1);
        Logger.RegisterTalon("Left Intake", leftIntake);
        Logger.RegisterTalon("Right Intake", rightIntake);
    }

    public void configureMotors() {
        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
        configs.SupplyCurrentLimitEnable = true;
        configs.SupplyCurrentLimit = 70;
        configs.SupplyCurrentThreshold = 90;
        configs.SupplyTimeThreshold = 0.2;

        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();

        slot0Configs.kS = 0.0; // no output overcome static friction
        slot0Configs.kV = 12./6000.; // 6000 RPM over 12V
        slot0Configs.kP = 0.011; // An error of 1 rpm results in 0.011 V output
        slot0Configs.kI = 0.00; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
        
        rightIntake.getConfigurator().apply(configs);
        rightIntake.getConfigurator().apply(slot0Configs);
        rightIntake.setInverted(false);

        leftIntake.getConfigurator().apply(configs);
        leftIntake.getConfigurator().apply(slot0Configs);
        leftIntake.setInverted(true);

    }

    public void setIntake(boolean isRunning, boolean inverted) {
        var power = isRunning ? (inverted ? -0.5 : 0.5) : 0.0;
        this.isRunning = isRunning;
        this.inverted = inverted;
        rightIntake.set(power*ratio);
        leftIntake.set(power);
    }

    public double getPercentOutput() {
        return rightIntake.get();
    }

    public boolean isRunning() {
        return isRunning;
    }

    public boolean isInverted() {
        return inverted;
    }

    public boolean isPieceSeen() {
        return !highSensor.get();
    }


    public void removeInterrupt() {
        interrupt = false;
        isRunning = true;
        setIntake(isRunning, inverted);
    }

    @Override
    public void updateInputs() {
        if(interrupt && isRunning) {
            isRunning = false;
            //setIntake(isRunning, inverted);
            return;
        }
    }

    @Override
    public void setPower(double power) {
        rightIntake.set(2*power*ratio);
        leftIntake.set(2*power);
    }

    @Override
    public void setRpm(double rpm) {
        rightIntake.setControl(pidRequest.withVelocity(2*rpm*ratio));
        leftIntake.setControl(pidRequest.withVelocity(2*rpm));

    }
}
