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
        speed.put(1.375, 5000.);
        speed.put(1.78, 5000.);
        speed.put(2.32, 5000.);
        speed.put(2.8, 5000.);
        speed.put(3.12, 5300.);
        speed.put(3.43, 5300.);
        speed.put(3.92, 5500.);

        //input: Meters from target, output: Degrees of shot  
        angle.put(1.375, 52.);
        angle.put(1.78, 48.);
        angle.put(2.05, 44.);
        angle.put(2.31, 41.);
        angle.put(2.55, 39.5);
        angle.put(2.8, 37.5);
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