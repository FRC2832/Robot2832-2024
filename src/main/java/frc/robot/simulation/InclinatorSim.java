package frc.robot.simulation;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import frc.robot.interfaces.IInclinatorHw;

public class InclinatorSim implements IInclinatorHw {
    private SimDevice climbDevice;
    private SimDouble leftPos, rightPos;
    private double leftPower, rightPower;

    //6380 rpm * 1/100 gear ratio * 2.536"/rev (0.75" spool diameter) * 1/60 min/sec
    final double kFactor = 2.505;

    public InclinatorSim() {
        climbDevice = SimDevice.create("Jukebox:Climber");
        leftPos = climbDevice.createDouble("Left Climb Pos", Direction.kOutput, 17);
        rightPos = climbDevice.createDouble("Right Climb Pos", Direction.kOutput, 17);
    }

    @Override
    public void setPower(double power) {
        leftPower = power;
        rightPower = power;
    }

    @Override
    public void setPower(double powerLeft, double powerRight) {
        leftPower = powerLeft;
        rightPower = powerRight;
    }

    @Override
    public void updateInputs() {
        leftPos.set(leftPos.get() + (leftPower * kFactor));
        rightPos.set(rightPos.get() + (rightPower * kFactor));
    } 
}
