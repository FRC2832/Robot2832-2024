package frc.robot.shooter;
import org.livoniawarriors.AutoShotLookup;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        speed.put(1.3, 4500.);
        speed.put(2., 4500.);
        speed.put(2.39, 4500.);
        speed.put(2.71, 4500.);
        speed.put(3.25, 4500.);

        //input: Meters from target, output: Degrees of shot  
        angle.put(1.3, 53.);
        angle.put(1.95, 49.);
        angle.put(2.26, 42.);
        angle.put(2.9, 38.);
        angle.put(3.13, 37.);
        angle.put(3.5, 35.);

        //input: Meters from target, output: RPM for kicker
        kicker.put(1.3, 4300.);
        kicker.put(2., 4500.);
        kicker.put(2.39, 4500.);
        kicker.put(2.71, 4500.);
        kicker.put(3.25, 4500.);
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

    /** @param speedDiff percent difference between -1 to 1 */
    public void spinShot(double speedDiff, boolean spinRight) {
        // taking in two parameters to determine difference in speed rotation and direction.
        var currentSpeed = SmartDashboard.getNumber("Shooter Command", 0); 
        if(spinRight) {
            currentSpeed -= speedDiff; // changing the speed
            for(int i = 1; i < 3; i++){ 
                if(i % 2 != 0) { //setting speed to even motors
                    hw.setIndividualPower(i, currentSpeed);
                }            
            }
        } else {
            currentSpeed -= speedDiff; // changing the speed
            for(int i = 1; i < 3; i++){
                if(i % 2 == 0) { //setting speed to even motors
                    hw.setIndividualPower(i, currentSpeed); 
                }            
            }
        }

        //Eventually use shoot function
    }

    public boolean canMakeShot() {
        //check distance, rpm, height of shooter, turned left/right
        return false;
    }

    public void autoShoot() {
        // Handle auto calculations and then shoot
    }
}