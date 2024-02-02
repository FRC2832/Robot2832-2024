package frc.robot.subsystems;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IShooterHw;

public class Shooter extends SubsystemBase {
    private IShooterHw hw;

    public Shooter(IShooterHw hardware) {
        super();
        hw = hardware;
        InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

        //input: Inches from target, output: RPM of shot  
        table.put(125.0, 450.0);
        table.put(200.0, 510.0);
        table.put(268.0, 525.0);
        table.put(312.0, 550.0);
        table.put(326.0, 650.0);

        var rpm = table.get(175.0);
    }
    
    @Override
    public void periodic() {
        hw.updateInputs();
    }
    
    public void setRPM(double RPM) {
        hw.setRpm(RPM);
    }

    public double getRPM() {
        return 0.00;
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