package frc.robot.kicker;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;

public class KickerSim extends Kicker {
    private SimDevice kickerDevice;
    private SimDouble leftRpmDev;
    private SimDouble rightRpmDev;
    private double leftRpm, rightRpm;
    private double leftTarget, rightTarget;
    private final double MAX_RPM = 6380;

    public KickerSim() {
        kickerDevice = SimDevice.create("Jukebox:Kicker");
        if(kickerDevice != null) {
            leftRpmDev = kickerDevice.createDouble("Left RPM", Direction.kOutput, 0);
            rightRpmDev = kickerDevice.createDouble("Right RPM", Direction.kOutput, 0);
        }
    }

    @Override
    public void setPower(double power) {
        leftTarget = power * MAX_RPM;
        rightTarget = power * MAX_RPM;
    }

    @Override
    public void setRpm(double rpm) {
        leftTarget = Math.copySign(Math.min(Math.abs(rpm),MAX_RPM), rpm);
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
        if(kickerDevice != null) {
            leftRpmDev.set(leftRpm);
            rightRpmDev.set(rightRpm);
        }
    }
}
