package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IInclinatorHw;

public class Inclinator extends SubsystemBase {
    private IInclinatorHw hardware;
    
    public Inclinator(IInclinatorHw hardware){
        this.hardware = hardware;
    }
    
    @Override
    public void periodic() {
        hardware.updateInputs();
    }

    /** @param position in inches */
    public void setPosition(double position) {
        
    }

    public double getPosition() {
        return 12.0;
    }

    public boolean isHomed() {
        return false;
    }

    public void home() {
        
    }

    public void setPower(double power) {
        hardware.setPower(power);
    }

    public void setOffsetPower(double power, boolean left){
        if(left){
            hardware.setPower(power, 0);
        } 
        else{
            hardware.setPower(0, power);
        }
    }

    public double getLeftCurrent(){
        return hardware.getLeftCurrent();
    }

    public double getRightCurrent(){
        return hardware.getRightCurrent();
    }
}
