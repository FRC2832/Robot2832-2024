package frc.robot.Controls;

import org.livoniawarriors.T16000M;
import org.livoniawarriors.UtilFunctions;
import edu.wpi.first.networktables.DoubleSubscriber;


/**
 * Drive the robot with T16000 controller 
 */
public class FlightDriveControls implements IDriveControls {

    private T16000M driveContLeft;
    private T16000M driveContRight;
    private DoubleSubscriber deadband;

    /**
     * Inject the drivetrain and controller to use
     * @param drive Drivetrain to command
     * @param driveContLeft Controller to read from
     * @param driveContRight Controller to read from
     */
    public FlightDriveControls() {

        this.driveContLeft = new T16000M(0);
        this.driveContRight = new T16000M(1);
        deadband = UtilFunctions.getSettingSub("DriveStick/Deadband", 0.02);
    }


    @Override
    public double GetXDrivePct() {
        var dead = deadband.get();
        return UtilFunctions.deadband(-driveContRight.getyAxis1(), dead);
    }

    @Override
    public double GetYDrivePct() {
        var dead = deadband.get();
        return UtilFunctions.deadband(-driveContRight.getxAxis1(), dead);
    }

    @Override
    public double GetTurnPct() {
        var dead = deadband.get();
        return -UtilFunctions.deadband(driveContLeft.getxAxis1(), dead);
    }

    @Override
    public boolean IsFieldOrientedResetRequested() {
        return driveContLeft.getRawButtonPressed(2);
    }


    @Override
    public boolean IsIntakeRequested() {
        return driveContLeft.getTrigger();
    }


    @Override
    public boolean IsMaxSpeedRequested() {
        return driveContRight.getTrigger();
    }

    @Override
    public boolean IsCenterFieldShotRequested() {
        return driveContLeft.getLeftSideDownRightPressed(); //TODO: Change this if it matters
    }

    @Override
    public boolean IsPillarShotRequested() {
        return driveContLeft.getLeftSideDownLeftPressed(); //TODO: Change this if it matters
    }
}
