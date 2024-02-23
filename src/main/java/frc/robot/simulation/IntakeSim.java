package frc.robot.simulation;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import frc.robot.interfaces.IIntakeHw;

public class IntakeSim implements IIntakeHw {
    private SimDevice shooterDevice;
    private SimBoolean isRunningDev;
    private SimBoolean invertedDev;
    private boolean isRunning, inverted;

    public IntakeSim() {
        shooterDevice = SimDevice.create("Jukebox:Intake");
        if(shooterDevice != null) {
            isRunningDev = shooterDevice.createBoolean("Intake Running?", Direction.kOutput, false);
            invertedDev = shooterDevice.createBoolean("Intake Inverted", Direction.kOutput, false);
        }
        isRunning = false;
        inverted = false;
    }
    
    @Override
    public void setIntake(boolean running, boolean isInverted) {
        isRunning = running;
        inverted = isInverted;
    }

    @Override
    public boolean isRunning() {
        return isRunning;
    }

    @Override
    public boolean isInverted() {
        return inverted;
    }

    @Override
    public boolean isPieceSeen() {
        return false;
    }

    @Override
    public double getPercentOutput() {
        return 0.0;
    }

    @Override
    public void removeInterrupt() { }

    @Override
    public void updateInputs() {
        if(shooterDevice != null) {
            isRunningDev.set(isRunning);
            invertedDev.set(inverted);
        }
    }

    @Override
    public void setPower(double power) {
        // TODO Auto-generated method stub
    }

    @Override
    public void setRpm(double rpm) {
        // TODO Auto-generated method stub
    }
    
}
