package frc.robot.simulation;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import frc.robot.interfaces.IIntakeHw;

public class IntakeSim implements IIntakeHw {
    private SimDevice shooterDevice;
    private SimDouble intakeRpm;
    private SimDouble midRpm;
    private double intakeTarget, midTarget;

    public IntakeSim() {
        shooterDevice = SimDevice.create("Jukebox:Intake");
        intakeRpm = shooterDevice.createDouble("Intake RPM", Direction.kOutput, 0);
        midRpm = shooterDevice.createDouble("Mid Wheel RPM", Direction.kOutput, 0);
    }
    
    @Override
    public void setIntakePower(double power) {
        intakeTarget = power * 6000.;
    }

    @Override
    public double getIntakeRpm() {
        return intakeRpm.get();
    }

    @Override
    public boolean isPieceSeen() {
        //TODO Implement piece seen
        return false;
    }

    @Override
    public void setMidPower(double power) {
        midTarget = power * 6000.;
    }

    @Override
    public double getMidRpm() {
        return midRpm.get();
    }

    @Override
    public void updateInputs() {
        intakeRpm.set(UtilFunctions.LimitChange(intakeRpm.get(), intakeTarget, 150));
        midRpm.set(UtilFunctions.LimitChange(midRpm.get(), midTarget, 150));
    }
    
}
