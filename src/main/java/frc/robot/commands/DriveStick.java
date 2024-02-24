package frc.robot.commands;


import org.livoniawarriors.swerve.SwerveDriveTrain;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.interfaces.IDriveControls;

/**
 * Drive the robot with joysticks 
 */
public class DriveStick extends Command {

    private SwerveDriveTrain drive;
    private IDriveControls cont;
    private InterpolatingDoubleTreeMap controlTableX;
    private InterpolatingDoubleTreeMap controlTableY;
    private InterpolatingDoubleTreeMap turnTable;
    private double xSpeed;
    private double ySpeed;
    private double turn;
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
        controlTableX = new InterpolatingDoubleTreeMap();
        controlTableY = new InterpolatingDoubleTreeMap();
        turnTable = new InterpolatingDoubleTreeMap();
        //Table for Smoother X Driving
        controlTableX.put(0.0,0.0);
        controlTableX.put(0.2,0.04);
        controlTableX.put(0.4,0.16);
        controlTableX.put(0.6,0.36);
        controlTableX.put(0.8,0.64);
        controlTableX.put(1.0,1.0);
        //Table for Smoother Y Driving
        controlTableY.put(0.0,0.0);
        controlTableY.put(0.2,0.04);
        controlTableY.put(0.4,0.16);
        controlTableY.put(0.6,0.36);
        controlTableY.put(0.8,0.64);
        controlTableY.put(1.0,1.0);
        //Table for Smoother turning
        turnTable.put(0.0,0.0);
        turnTable.put(0.2,0.04);
        turnTable.put(0.4,0.16);
        turnTable.put(0.6,0.36);
        turnTable.put(0.8,0.64);
        turnTable.put(1.0,1.0);
    }

    @Override
    public void execute() {
        if (cont.IsFieldOrientedResetRequested()) {
            drive.resetFieldOriented();
        }
        
        if(cont.GetXDrivePct()>=0){
            xSpeed = controlTableX.get(Math.abs(cont.GetXDrivePct()));
        }
        else{
            xSpeed = -controlTableX.get(Math.abs(cont.GetXDrivePct()));
        }
        if(cont.GetYDrivePct()>=0){
            ySpeed = controlTableY.get(Math.abs(cont.GetYDrivePct()));
        }
        else{
            ySpeed = -controlTableY.get(Math.abs(cont.GetYDrivePct()));
        }
        if(cont.GetTurnPct()>=0){
            turn = turnTable.get(Math.abs(cont.GetTurnPct()));
        }
        else{
            turn = -turnTable.get(Math.abs(cont.GetTurnPct()));
        }
        
        xSpeed = cont.GetXDrivePct();
        ySpeed = cont.GetYDrivePct();
        turn = cont.GetTurnPct();
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
