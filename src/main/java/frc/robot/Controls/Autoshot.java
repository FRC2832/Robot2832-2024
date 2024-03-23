package frc.robot.Controls;
import frc.robot.intake.Intake;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;
import frc.robot.aimer.Pneumatics;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.livoniawarriors.swerve.SwerveDriveTrain;
import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.odometry.Odometry;

public class Autoshot extends Command {
    Shooter shooter;
    Pneumatics pneumatic;
    Kicker kicker;
    Odometry odometry;
    Intake intake;
    Double tagY;
    int goodCounts;
    SwerveDriveTrain swerveDrive;
    PIDController pid;
    public Autoshot(Shooter shooter, Pneumatics pneumatic, Kicker kicker, Odometry odometry, Intake intake, SwerveDriveTrain  swerveDrive) {
        this.shooter = shooter;
        this.pneumatic = pneumatic;
        this.kicker = kicker;
        this.odometry = odometry;
        this.intake = intake;
        this.tagY = 218.42 * 0.0254;
        this.swerveDrive = swerveDrive;
        this.pid = new PIDController(.6/Math.PI, .1, 0); //DID NOT THINK ABOUT SETTINGS!!!
        addRequirements(shooter, pneumatic, kicker, intake, swerveDrive);

    }

    @Override
    public void initialize() {
        goodCounts = 0;
    }

    @Override 
    public void execute(){
        var tagX = (UtilFunctions.getAlliance() == Alliance.Red ? 652.73 : -1.5) * 0.0254;

        var robotPose = odometry.getPose();
        var speakerPose = new Pose2d(tagX, tagY, null);
        //double inverse = 1;
        double inverse = ((robotPose.getX()-speakerPose.getX())>0)?-1.0 : 1.0;
        var distance = UtilFunctions.getDistance(speakerPose, robotPose); //TODO: Need to handle rotation
        var angleDiff = UtilFunctions.getAngle(speakerPose, robotPose) - odometry.getHeading().getRadians();
        System.out.println("HI");

        System.out.println(angleDiff);
        if(UtilFunctions.getAlliance() == Alliance.Blue){
            angleDiff = Math.PI-angleDiff;
        }
        angleDiff = pid.calculate(odometry.getHeading().getRadians(), UtilFunctions.getAngle(speakerPose, robotPose));
        swerveDrive.SwerveDrive(0,0,angleDiff);
        

        AutoShotLookup lookup = shooter.estimate(distance);

        shooter.setRPM(lookup.getShooterSpeed());
        pneumatic.goTo(lookup.getAngle());
        kicker.setRPM(lookup.getKickerSpeed());
        if (  (  (Math.abs(shooter.getRPM() - lookup.getShooterSpeed()) < 75)
              && (Math.abs(pneumatic.getAngle() - lookup.getAngle()) < 4)
              && (Math.abs(kicker.getRPM()-lookup.getKickerSpeed())<75)
              )
           )
        {
            intake.setRpm(250);
            //wait to start timeout until piece leaves starts leaving intake
            if(intake.isPieceDetected() == false) {
                goodCounts++;
            }   
        }
        else {
            intake.setPower(0);
        }
    }

    @Override
    public boolean isFinished() {
        return goodCounts > 40;
    }

    @Override
    public void end(boolean interrupted) {
        pneumatic.stop();
        intake.setPower(0);
        //keep shooter running in auto
        shooter.setRPM(3000);
        kicker.stop();
    }
}