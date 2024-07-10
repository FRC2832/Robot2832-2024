package frc.robot.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    public abstract void updateInputs();
    public abstract void setPower(double power);
    public abstract void setRpm(double rpm);
    public abstract double getCurrentRPM(int shooterID);

    public Shooter() {
        super();
    }

    @Override
    public void periodic() {
        updateInputs();
    }

    public double getRpm() {
        return getCurrentRPM(0);
    }

    
    public static double RPMToVelocity(double RPM) {
        return ((((RPM*(Math.PI)*2)/60)*2)/(39.3700787402));
    }

    public boolean isAtSpeed() {
        return false;
    }

    public void shoot() {
        // if shooter is at x speed and note is detected?
    }

    public boolean canMakeShot() {
        //check distance, rpm, height of shooter, turned left/right
        return false;
    }

    public void autoShoot() {
        // Handle auto calculations and then shoot
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
        return runOnce(() -> setRpm(rpm))
            .withName("ShooterStart");
    }

    /** 
     * Run the shooter at an rpm 
     * @param rpm RPM to run at
     * @return Command
    */
    public Command runShooter(DoubleSupplier rpm) {
        return run(() -> setRpm(rpm.getAsDouble()))
            .finallyDo(() -> setPower(0))
            .withName("ShooterRun");
    }

    /**
     * Runs the shooter in reverse
     * @return Command
     */
    public Command reverseShooter() {
        return run(() -> setRpm(-3000))
            .finallyDo(interupt -> setPower(0))
            .withName("ShooterReverse");
    }
}