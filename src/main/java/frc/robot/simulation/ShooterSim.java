package frc.robot.simulation;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import frc.robot.interfaces.IShooterHw;

public class ShooterSim implements IShooterHw {
    private SimDevice shooterDevice;
    private SimDouble leftRpm;
    private SimDouble rightRpm;
    private double leftTarget, rightTarget;

    public ShooterSim() {
        shooterDevice = SimDevice.create("Jukebox:Shooter");
        leftRpm = shooterDevice.createDouble("Left RPM", Direction.kOutput, 0);
        rightRpm = shooterDevice.createDouble("Right RPM", Direction.kOutput, 0);
    }

    @Override
    public void setPower(double power) {
        leftTarget = power * 6450.;
        rightTarget = power * 6450.;
    }

    @Override
    public void setRpm(double rpm) {
        leftTarget = Math.copySign(Math.min(Math.abs(rpm),6450.), rpm);
        rightTarget = leftTarget;
    }

    @Override
    public double getCurrentRPM(int shooterID) {
        if(shooterID % 2 == 0) {
            return leftRpm.get();
        } else {
            return rightRpm.get();
        }
    }

    @Override
    public void setIndividualPower(int shooterID, double newPower) {
        if(shooterID % 2 == 0) {
            leftTarget = newPower * 6450.;
        } else {
            rightTarget = newPower * 6450.;
        }
    }

    @Override
    public void updateInputs() {
        leftRpm.set(UtilFunctions.LimitChange(leftRpm.get(), leftTarget, 150));
        rightRpm.set(UtilFunctions.LimitChange(rightRpm.get(), rightTarget, 150));
    }
}
