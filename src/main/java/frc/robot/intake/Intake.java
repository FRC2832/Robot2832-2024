package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class Intake extends SubsystemBase {
    public static final double PIT_INTAKE_SPEED = 225;
    /** @param inverted true = spin backwards, false = spin forwards */
    public abstract void setIntake(boolean isRunning, boolean inverted);
    public abstract double getPercentOutput();
    public abstract boolean isRunning();
    public abstract boolean isInverted();
    public abstract boolean isPieceSeen();
    public abstract void removeInterrupt();
    public abstract void updateInputs();
    public abstract void setPower(double power);
    public abstract void setRpm(double rpm);

    DoubleSubscriber intakeSpeed;

    public Intake() {
        super();
        intakeSpeed = UtilFunctions.getSettingSub("/DriveIntake/Intake Speed", 150);
    }

    @Override
    public void periodic() {
        updateInputs();
        SmartDashboard.putNumber("Intake Percent", getPercentOutput());
    }

    public boolean isPieceDetected() {
        return isPieceSeen();
    }

    public void stop() {
        setPower(0);
    }

    /* runs the intake at the default speed */
    public Command drive(boolean stopOnPiece) {
        return drive(intakeSpeed, stopOnPiece, false);
    }

    /* runs the intake at the default speed */
    public Command drive(boolean stopOnPiece, boolean invertDir) {
        return drive(intakeSpeed, stopOnPiece, invertDir);
    }

    /* runs the intake at a custom speed */
    public Command drive(DoubleSupplier speed, boolean stopOnPiece) {
        return drive(speed, stopOnPiece, false);
    }

    public Command drive(DoubleSupplier speed, boolean stopOnPiece, boolean invertDir) {
        return run(() -> {
            double dir = 1;
            if(invertDir) dir = -1;
            setRpm(dir * intakeSpeed.get());    
        })
        .until(() -> isPieceDetected() && stopOnPiece)
        .finallyDo(this::stop)
        .withName("IntakeDrive");
    }
}
