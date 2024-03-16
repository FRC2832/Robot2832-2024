package frc.robot.shooter;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;

public class ShooterSim implements IShooterHw {
    private SimDevice shooterDevice;
    private SimDouble leftRpmDev;
    private SimDouble rightRpmDev;
    private double leftRpm, rightRpm;
    private double leftTarget, rightTarget;

    public ShooterSim() {
        shooterDevice = SimDevice.create("Jukebox:Shooter");
        if(shooterDevice != null) {
            leftRpmDev = shooterDevice.createDouble("Left RPM", Direction.kOutput, 0);
            rightRpmDev = shooterDevice.createDouble("Right RPM", Direction.kOutput, 0);
        }
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
            return leftRpm;
        } else {
            return rightRpm;
        }
    }


    @Override
    public void updateInputs() {
        leftRpm = UtilFunctions.LimitChange(leftRpm, leftTarget, 150);
        rightRpm = UtilFunctions.LimitChange(rightRpm, rightTarget, 150);
        if(shooterDevice != null) {
            leftRpmDev.set(leftRpm);
            rightRpmDev.set(rightRpm);
        }
    }
}
