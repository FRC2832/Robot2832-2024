package frc.robot.Controls;

import org.livoniawarriors.UtilFunctions;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.XboxController;


/**
 * Drive the robot with T16000 controller 
 */
public class XboxDriveControls implements IDriveControls {

    private XboxController cont;

    private DoubleSubscriber deadband;

    /**
     * Inject the drivetrain and controller to use
     * @param drive Drivetrain to command
     * @param cont Controller to read from
     */
    public XboxDriveControls() {

        this.cont = new XboxController(0);

        deadband = UtilFunctions.getSettingSub("DriveXbox/Deadband", 0.13);
        
    }


    @Override
    public double GetXDrivePct() {
        var dead = deadband.get();
        return UtilFunctions.deadband(-cont.getLeftY(), dead);
    }

    @Override
    public double GetYDrivePct() {
        var dead = deadband.get();
        return UtilFunctions.deadband(-cont.getLeftX(), dead);
    }

    @Override
    public double GetTurnPct() {
        var dead = deadband.get();
        return UtilFunctions.deadband(-cont.getRightX(), dead);
    }

    @Override
    public boolean IsFieldOrientedResetRequested() {
        return cont.getLeftStickButtonPressed();
    }


    @Override
    public boolean IsIntakeRequested() {
        return cont.getLeftTriggerAxis()>0;
    }


    @Override
    public boolean IsMaxSpeedRequested() {
        return cont.getRightTriggerAxis()>0;
    }

    @Override
    public boolean IsCenterFieldShotRequested() {
        return cont.getBButtonPressed();
    }

    @Override
    public boolean IsPillarShotRequested() {
        return cont.getXButtonPressed();
    }
    
}
