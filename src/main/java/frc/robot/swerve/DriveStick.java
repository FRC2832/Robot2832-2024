package frc.robot.swerve;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls.IDriveControls;

/**
 * Drive the robot with joysticks 
 */
public class DriveStick extends Command {

    private SwerveSubsystem drive;
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
    public DriveStick(SwerveSubsystem drive, IDriveControls cont) {
        this.drive = drive;
        this.cont = cont;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.drive(new ChassisSpeeds(0, 0, 0));
        controlTableX = new InterpolatingDoubleTreeMap();
        controlTableY = new InterpolatingDoubleTreeMap();
        turnTable = new InterpolatingDoubleTreeMap();
        //Table for Smoother X Driving
        controlTableX.put(0.0,0.0);
        controlTableX.put(0.2,0.25);
        controlTableX.put(0.4,0.45);
        controlTableX.put(0.6,0.7);
        controlTableX.put(0.8,0.9);
        controlTableX.put(1.0,1.0);
        //Table for Smoother Y Driving
        controlTableY.put(0.0,0.0);
        controlTableY.put(0.2,0.25);
        controlTableY.put(0.4,0.45);
        controlTableY.put(0.6,0.7);
        controlTableY.put(0.8,0.9);
        controlTableY.put(1.0,1.0);
        //Table for Smoother turning
        turnTable.put(0.0,0.0);
        turnTable.put(0.2,0.2);
        turnTable.put(0.4,0.4);
        turnTable.put(0.6,0.6);
        turnTable.put(0.8,0.75);
        turnTable.put(1.0,1.0);
    }

    @Override
    public void execute() {
        if (cont.IsFieldOrientedResetRequested()) {
            drive.zeroGyro();
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
        
        //make sure the drive command stays in the circle of max speed
        //xSpeed + ySpeed could be bigger as they are independent
        double maxSpeed = drive.getMaxDriverSpeed();
        double xDrive = xSpeed * maxSpeed;
        double yDrive = ySpeed * maxSpeed;
        double hyp = Math.pow((xDrive * xDrive) + (yDrive * yDrive), 0.5);

        //if faster than max speed, divide out the ratio too fast 
        if(hyp > maxSpeed){
            xDrive = xDrive * (maxSpeed / hyp);
            yDrive = yDrive * (maxSpeed / hyp);
        }

        drive.drive(new Translation2d(xDrive, yDrive), turn * drive.getMaxDriverOmega(), true);
    }

    @Override
    public boolean isFinished() {
        //never end
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
