package frc.robot.hardware;

import frc.robot.interfaces.IIntakeHw;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.DigitalInput;

@SuppressWarnings("removal")
public class IntakeHw implements IIntakeHw {
    private TalonFX hardware;
    private DigitalInput enterSensor;
    private boolean isRunning;
    private boolean inverted;
    private boolean interrupt;

    public IntakeHw() {
        this.hardware = new TalonFX(50);
        this.enterSensor = new DigitalInput(0);
        this.isRunning = false;
        this.inverted = false;
        this.interrupt = false;

        SmartDashboard.putBoolean("Note In", false);
    }

    public void setIntake(boolean isRunning, boolean inverted) {
        var power = isRunning ? (inverted ? -1.0 : 1.0) : 0.0;
        this.isRunning = isRunning;
        this.inverted = inverted;
        hardware.set(TalonFXControlMode.PercentOutput, power);
    }

    public double getPercentOutput() {
        return hardware.getMotorOutputPercent();
    }

    public boolean isRunning() {
        return isRunning;
    }

    public boolean isInverted() {
        return inverted;
    }

    public boolean isPieceSeen() {
        return false;
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
            setIntake(isRunning, inverted);
            return;
        }
        if(enterSensor.get()) {
            interrupt = true;
            SmartDashboard.putBoolean("Note In", true);
            return;
        }
    }
}
