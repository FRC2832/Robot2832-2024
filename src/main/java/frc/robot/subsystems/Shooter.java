package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Shooter {
    public Shooter() {
        InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

        //input: Inches from target, output: RPM of shot  
        table.put(125.0, 450.0);
        table.put(200.0, 510.0);
        table.put(268.0, 525.0);
        table.put(312.0, 550.0);
        table.put(326.0, 650.0);

        var rpm = table.get(175.0);
    }
    
    public void setRPM(double RPM) {
        
    }
    public boolean isAtSpeed() {
        return false;
    }
    
    /**@param power percent -1 to 1 */
    public void setPower (double power) {
        
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