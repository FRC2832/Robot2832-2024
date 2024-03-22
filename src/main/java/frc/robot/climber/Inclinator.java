package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Inclinator extends SubsystemBase {
    private IInclinatorHw hardware;
    private double leftRequest, rightRequest;

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
        leftRequest = power;
        rightRequest = power;
    }

    public void setOffsetPower(double power, boolean left){
        if(left){
            leftRequest = power;
        } 
        else{
            rightRequest = power;
        }
    }

    public double getLeftCurrent(){
        return hardware.getLeftCurrent();
    }

    public double getRightCurrent(){
        return hardware.getRightCurrent();
    }

    public boolean getLeftLimit() {
        return hardware.getLeftLimit();
    }

    public boolean getRightLimit() {
        return hardware.getRightLimit();
    }

    public double getLeftRequest() {
        return leftRequest;
    }

    public double getRightRequest() {
        return rightRequest;
    }

    public void setArbRequest(double left, double right) {
        hardware.setPower(left, right);
    }

    public double getLeftHeight() {
        return hardware.getLeftHeight();
    }

    public double getRightHeight() {
        return hardware.getRightHeight();
    }
}
