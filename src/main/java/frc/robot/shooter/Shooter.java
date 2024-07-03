package frc.robot.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls.AutoShotLookup;

public abstract class Shooter extends SubsystemBase {
    private InterpolatingDoubleTreeMap speed, angle, kicker;

    public abstract void updateInputs();
    public abstract void setPower(double power);
    public abstract void setRpm(double rpm);
    public abstract double getCurrentRPM(int shooterID);

    public Shooter() {
        super();
        
        speed = new InterpolatingDoubleTreeMap();
        angle = new InterpolatingDoubleTreeMap();
        kicker = new InterpolatingDoubleTreeMap();

        //input: Meters from target, output: RPM of shot  
        speed.put(1.375, 5000.);
        speed.put(1.78, 5000.);
        speed.put(2.32, 5000.);
        speed.put(2.8, 5000.);
        speed.put(3.12, 5300.);
        speed.put(3.43, 5300.);
        speed.put(3.92, 5500.);

        //input: Meters from target, output: Degrees of shot  
        angle.put(1.375, 54.);
        angle.put(1.78, 50.);
        angle.put(2.32, 46.);
        angle.put(2.8, 39.);
        angle.put(3.12, 36.);
        angle.put(3.43, 32.);
        angle.put(3.92, 28.);

        //input: Meters from target, output: RPM for kicker
        kicker.put(1.375, 6000.);
        kicker.put(1.78, 6000.);
        kicker.put(2.32, 6000.);
        kicker.put(2.8, 6000.);
        kicker.put(3.12, 6000.);
        kicker.put(3.43, 6000.);
        kicker.put(3.92, 6000.);
    }

    @Override
    public void periodic() {
        updateInputs();
    }

    public double getRpm() {
        return getCurrentRPM(0);
    }

    public AutoShotLookup estimate(double d) {
        AutoShotLookup shot = new AutoShotLookup(angle.get(d), kicker.get(d), speed.get(d));
        shot.printValues();
        return shot;
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
     * Runs the shooter in reverse
     * @return Command
     */
    public Command reverseShooter() {
        return run(() -> setRpm(-3000))
            .finallyDo(interupt -> setPower(0))
            .withName("ShooterReverse");
    }
}