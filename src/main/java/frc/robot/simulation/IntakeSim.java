package frc.robot.simulation;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import frc.robot.interfaces.IIntakeHw;

public class IntakeSim implements IIntakeHw {
    private SimDevice shooterDevice;
    private SimBoolean isRunning;
    private SimBoolean inverted;

    public IntakeSim() {
        shooterDevice = SimDevice.create("Jukebox:Intake");
        isRunning = shooterDevice.createBoolean("Intake Running?", Direction.kOutput, false);
        inverted = shooterDevice.createBoolean("Intake Inverted", Direction.kOutput, false);
    }
    
    @Override
    public void setIntake(boolean running, boolean isInverted) {
        isRunning.set(running);
        inverted.set(isInverted);
    }

    @Override
    public boolean isRunning() {
        return isRunning.get();
    }

    @Override
    public boolean isInverted() {
        return inverted.get();
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

    }
    
}
