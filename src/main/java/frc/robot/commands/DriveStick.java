package frc.robot.commands;


import org.livoniawarriors.swerve.SwerveDriveTrain;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.interfaces.IDriveControls;
import frc.robot.subsystems.Intake;


/**
 * Drive the robot with joysticks 
 */
public class DriveStick extends Command {

    private SwerveDriveTrain drive;
    private Intake intake;
    private IDriveControls cont;
    private InterpolatingDoubleTreeMap controlTable;
    private InterpolatingDoubleTreeMap turnTable;
    private double xSpeed;
    private double ySpeed;
    private double turn;
    /**
     * Inject the drivetain and controller to use
     * @param drive Drivetrain to command
     * @param cont Controller to read from
     */
    public DriveStick(SwerveDriveTrain drive, IDriveControls cont, Intake intake) {
        this.drive = drive;
        this.cont = cont;
        this.intake = intake;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Turtle Turn Speed", 4);
        drive.SwerveDrive(0, 0, 0, false);
        controlTable = new InterpolatingDoubleTreeMap();
        turnTable = new InterpolatingDoubleTreeMap();
        //Table for Smoother Driving
        controlTable.put(0.0,0.0);
        controlTable.put(0.2,0.04);
        controlTable.put(0.4,0.16);
        controlTable.put(0.6,0.36);
        controlTable.put(0.8,0.64);
        controlTable.put(1.0,1.0);
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
        if (cont.IsIntakeRequested()){
            intake.setIntake(true,false);
        }
        else{
            intake.setIntake(false,false);
        }
        /*
        if(cont.GetXDrivePct()>=0){
            xSpeed = controlTable.get(Math.abs(cont.GetXDrivePct()));
        }
        else{
            xSpeed = -controlTable.get(Math.abs(cont.GetXDrivePct()));
        }
        if(cont.GetYDrivePct()>=0){
            ySpeed = controlTable.get(Math.abs(cont.GetYDrivePct()));
        }
        else{
            ySpeed = -controlTable.get(Math.abs(cont.GetYDrivePct()));
        }
        if(cont.GetTurnPct()>=0){
            turn = turnTable.get(Math.abs(cont.GetTurnPct()));
        }
        else{
            turn = -turnTable.get(Math.abs(cont.GetTurnPct()));
        }
        */
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
