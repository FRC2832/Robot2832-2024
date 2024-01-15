package org.livoniawarriors.swerve;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive the robot with xbox controller 
 */
public class DriveXbox extends Command {
    private SwerveDriveTrain drive;
    private XboxController cont;
    private DoubleSubscriber deadband;

    /**
     * Inject the drivetrain and controller to use
     * @param drive Drivetrain to command
     * @param cont Controller to read from
     */
    public DriveXbox(SwerveDriveTrain drive, XboxController cont) {
        this.drive = drive;
        this.cont = cont;
        deadband = UtilFunctions.getSettingSub("DriveXbox/Deadband", 0.13);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.SwerveDrive(0, 0, 0, false);
    }

    @Override
    public void execute() {
        //driver clicked field reset stick
        if (cont.getLeftStickButtonPressed()) {
            drive.resetFieldOriented();
        }

        var dead = deadband.get();
        double xSpeed = UtilFunctions.deadband(-cont.getLeftY(), dead);
        double ySpeed = UtilFunctions.deadband(-cont.getLeftX(), dead);
        double turn   = UtilFunctions.deadband(-cont.getRightX(), dead);
        drive.SwerveDrive(
            xSpeed * drive.getMaxDriverSpeed(), 
            ySpeed * drive.getMaxDriverSpeed(), 
            turn * drive.getMaxDriverOmega()
        );
    }

    @Override
    public boolean isFinished() {
        //never end
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
