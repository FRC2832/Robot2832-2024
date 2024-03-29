package frc.robot.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Controls.AutoShotLookup;

public class Shooter extends SubsystemBase {
    private IShooterHw hw;
    private InterpolatingDoubleTreeMap speed, angle, kicker;

    public Shooter(IShooterHw hardware) {
        super();
        hw = hardware;
        
        speed = new InterpolatingDoubleTreeMap();
        angle = new InterpolatingDoubleTreeMap();
        kicker = new InterpolatingDoubleTreeMap();

        //input: Meters from target, output: RPM of shot  
        speed.put(1.3, 3500.);
        speed.put(2., 3500.);
        speed.put(2.39, 3500.);
        speed.put(2.71, 3500.);
        speed.put(3.25, 3500.);

        //input: Meters from target, output: Degrees of shot  
        angle.put(1.375, 54.);
        angle.put(1.84, 51.);
        angle.put(2.32, 41.);
        angle.put(2.8, 36.);
        angle.put(3.13, 30.);
        angle.put(3.5, 30.);

        //input: Meters from target, output: RPM for kicker
        kicker.put(1.3, 6000.);
        kicker.put(2., 6000.);
        kicker.put(2.39, 6000.);
        kicker.put(2.71, 6000.);
        kicker.put(3.25, 6000.);
    }

    @Override
    public void periodic() {
        hw.updateInputs();
    }
    public AutoShotLookup estimate(double d) {
        AutoShotLookup shot = new AutoShotLookup(angle.get(d), kicker.get(d), speed.get(d));
        shot.printValues();
        return shot;
    }

    public void setRPM(double RPM) {
        hw.setRpm(RPM);
    }

    public double getRPM() {
        return hw.getCurrentRPM(0);
    }
    
    public static double RPMToVelocity(double RPM) {
        return ((((RPM*(Math.PI)*2)/60)*2)/(39.3700787402));
    }

    public boolean isAtSpeed() {
        return false;
    }
    
    /**@param power percent -1 to 1 */
    public void setPower (double power) {
        hw.setPower(power);
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
}