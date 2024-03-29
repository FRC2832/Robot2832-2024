package frc.robot.climber;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import frc.robot.Robot;

public class InclinatorSim implements IInclinatorHw {
    private SimDevice climbDevice;
    private SimDouble leftPosDev, rightPosDev;
    private double leftPos, rightPos;
    private double leftPower, rightPower;

    //6380 rpm * 1/100 gear ratio * 2.536"/rev (0.75" spool diameter) * 1/60 min/sec
    final double kFactor = 2.505;

    public InclinatorSim() {
        climbDevice = SimDevice.create("Jukebox:Climber");
        if(climbDevice != null) {
            leftPosDev = climbDevice.createDouble("Left Climb Pos", Direction.kOutput, 17);
            rightPosDev = climbDevice.createDouble("Right Climb Pos", Direction.kOutput, 17);
        }
        leftPos = 17;
        rightPos = 17;
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
        leftPos = leftPos + (leftPower * kFactor * Robot.kDefaultPeriod);
        rightPos = rightPos + (rightPower * kFactor * Robot.kDefaultPeriod);
        if(climbDevice != null) {
            leftPosDev.set(leftPos);
            rightPosDev.set(rightPos);
        }
    }

    @Override
    public double getLeftCurrent() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRightCurrent() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean getLeftLimit() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean getRightLimit() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public double getLeftHeight() {
        return leftPos;
    }

    @Override
    public double getRightHeight() {
        return rightPos;
    } 
}
