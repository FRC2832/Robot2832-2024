package frc.robot.hardware;

import frc.robot.interfaces.IIntakeHw;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;


public class IntakeHw implements IIntakeHw {
    private TalonFX hardware;
    private boolean isRunning;
    private boolean inverted;

    public IntakeHw() {
        //TODO: can set config for this later
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        this.hardware = new TalonFX(50);
        this.isRunning = false;
        this.inverted = false;
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

    @Override
    public void updateInputs() {
        
    }
}
