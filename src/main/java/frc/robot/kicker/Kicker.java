package frc.robot.kicker;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Kicker extends SubsystemBase {
    private IKickerHw hw;

    public Kicker(IKickerHw hardware) {
        super();
        hw = hardware;
    }
    
    @Override
    public void periodic() {
        hw.updateInputs();
    }
    
    public void kick(double ShooterRPM) {
        hw.setRpm(ShooterRPM*24/18);
    }
    public void stop() {
        hw.setRpm(-10);
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

    public void setRPM(double rpm) {
        hw.setRpm(rpm);
    }
   }