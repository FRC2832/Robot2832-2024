package frc.robot.commands;


import org.livoniawarriors.swerve.SwerveDriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.interfaces.IDriveControls;


/**
 * Drive the robot with joysticks 
 */
public class DriveStick extends Command {

    private SwerveDriveTrain drive;
    private IDriveControls cont;


    /**
     * Inject the drivetain and controller to use
     * @param drive Drivetrain to command
     * @param cont Controller to read from
     */
    public DriveStick(SwerveDriveTrain drive, IDriveControls cont) {
        this.drive = drive;
        this.cont = cont;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Turtle Turn Speed", 4);
        drive.SwerveDrive(0, 0, 0, false);
    }

    @Override
    public void execute() {
        if (cont.IsFieldOrientedResetRequested()) {
            drive.resetFieldOriented();
        }

        
        double xSpeed = cont.GetXDrivePct();
        double ySpeed = cont.GetYDrivePct();
        double turn   = cont.GetTurnPct();
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
