package frc.robot.Controls;

import frc.robot.interfaces.IDriveControls;
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
    public FlightDriveControls(T16000M driveContLeft, T16000M driveContRight ) {

        this.driveContLeft = driveContLeft;
        this.driveContRight = driveContRight;
        deadband = UtilFunctions.getSettingSub("DriveXbox/Deadband", 0.13);
        
    }


    @Override
    public double GetXDrivePct() {
        var dead = deadband.get();
        return UtilFunctions.deadband(driveContRight.getyAxis1(), dead);
    }

    @Override
    public double GetYDrivePct() {
        var dead = deadband.get();
        return UtilFunctions.deadband(driveContRight.getxAxis1(), dead);
    }

    @Override
    public double GetTurnPct() {
        var dead = deadband.get();
        return UtilFunctions.deadband(driveContLeft.getxAxis1(), dead);
    }

    @Override
    public boolean IsFieldOrientedResetRequested() {
        return driveContLeft.getRawButtonPressed(2);
    }
}
