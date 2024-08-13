package frc.robot.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    public abstract void updateInputs();
    public abstract void setPower(double power);
    protected abstract void setRpmHw(double rpm);
    public abstract double getCurrentRPM(int shooterID);

    private BooleanEntry atRpm;
    private DoubleSubscriber errorRpm;
    private DoublePublisher rpmTarget;

    public Shooter() {
        super();
        atRpm = UtilFunctions.getNtEntry("/Shooter/At Rpm", false);
        rpmTarget = UtilFunctions.getNtPub("/Shooter/RPM Target", 0.0);
        errorRpm = UtilFunctions.getSettingSub("/Shooter/Error RPM Allowed", 300);
    }

    @Override
    public void periodic() {
        updateInputs();
    }

    public double getRpm() {
        return getCurrentRPM(0);
    }

    public void setRpm(double rpm) {
        setRpmHw(rpm);
    }
    
    public static double RPMToVelocity(double RPM) {
        return ((((RPM*(Math.PI)*2)/60)*2)/(39.3700787402));
    }

    /** 
     * Starts the shooter and keeps the rpm on exit 
     * @return Command
    */
    public Command startShooter() {
        return startShooter(6000);
    }

    /** 
     * Starts the shooter and keeps the rpm on exit 
     * @param rpm RPM to run at
     * @return Command
    */
    public Command startShooter(double rpm) {
        return run(() -> setRpm(rpm))
            .withName("ShooterStart");
    }

    /** 
     * Starts the shooter and keeps the rpm on exit 
     * @param rpm RPM to run at
     * @return Command
    */
    public Command startShooter(DoubleSupplier rpm) {
        return run(() -> setRpm(rpm.getAsDouble()))
            .withName("ShooterStart");
    }

    /** 
     * Run the shooter at an rpm 
     * @param rpm RPM to run at
     * @return Command
    */
    public Command runShooter(DoubleSupplier rpm) {
        return run(() -> {
            double rpmVal = rpm.getAsDouble();
            setRpm(rpmVal);
            updateAtRpm(rpmVal);
            rpmTarget.set(rpmVal);
        })
        .finallyDo(() -> {
            stopShooter();
            atRpm.set(false);
            rpmTarget.set(0);
        })
        .withName("ShooterRun");
    }

    /**
     * Runs the shooter in reverse
     * @return Command
     */
    public Command reverseShooter() {
        return run(() -> {
            setRpm(-3000);
        })
        .finallyDo(this::stopShooter)
        .withName("ShooterReverse");
    }

    public void stopShooter() {
        setPower(0);
    }
    
    public BooleanSupplier atRpm() {
        return atRpm;
    }

    private void updateAtRpm(double targetRpm) {
        double shooterError = Math.abs(targetRpm - getCurrentRPM(0));
        atRpm.set(shooterError < errorRpm.get());
    }
}