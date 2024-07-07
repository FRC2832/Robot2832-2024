package frc.robot.Controls;
import frc.robot.intake.Intake;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;
import frc.robot.aimer.Aimer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.odometry.Odometry;
import org.livoniawarriors.swerve.SwerveDriveTrain;

public class Autoshot extends Command {
    Shooter shooter;
    Aimer aimer;
    Kicker kicker;
    Odometry odometry;
    Intake intake;
    Double tagY;
    SwerveDriveTrain swerveDrive;
    PIDController pid;
    int goodCounts;

    public Autoshot(Shooter shooter, Aimer aimer, Kicker kicker, Odometry odometry, Intake intake, SwerveDriveTrain swerveDrive) {
        this.shooter = shooter;
        this.aimer = aimer;
        this.kicker = kicker;
        this.odometry = odometry;
        this.intake = intake;
        this.tagY = 218.42 * 0.0254;
        this.swerveDrive = swerveDrive;
        this.pid = new PIDController(.35/Math.PI, .45, 0); //DID NOT THINK ABOUT SETTINGS!!!
        addRequirements(shooter, aimer, kicker, intake, swerveDrive);
    }

    @Override
    public void initialize() {
        goodCounts = 0;
    }

    @Override 
    public void execute(){
        var alliColor = UtilFunctions.getAlliance();
        var tagX = (alliColor == Alliance.Red ? 652.73 : -1.5) * 0.0254;

        var robotPose = odometry.getPose();
        var speakerPose = new Pose2d(tagX, tagY, new Rotation2d());
        double angleToSpeakerRad = UtilFunctions.getAngle(speakerPose, robotPose);
        double targetAngleRad;

        if(alliColor == Alliance.Blue){
            targetAngleRad = angleToSpeakerRad;
        } else {
            targetAngleRad = Math.PI + angleToSpeakerRad;
        }

        //put robot angle in reference to target angle
        double robotAngleRad = robotPose.getRotation().getRadians();
        robotAngleRad = MathUtil.inputModulus(robotAngleRad, targetAngleRad - Math.PI, targetAngleRad + Math.PI);
        
        double angleCommand = pid.calculate(robotAngleRad, targetAngleRad);
        angleCommand = Math.toDegrees(angleCommand);
        SmartDashboard.putNumber("Aim Command", angleCommand);
        swerveDrive.SwerveDrive(0,0,angleCommand);
        //swerveDrive.SwerveDrive(0,0,-0);
        
        //calculate robot distance
        var distance = UtilFunctions.getDistance(new Pose2d(tagX, tagY, null), robotPose);
        AutoShotLookup lookup = shooter.estimate(distance);

        shooter.setRpm(lookup.getShooterSpeed());
        aimer.goToSmooth(lookup.getAngle());
        kicker.setRpm(lookup.getKickerSpeed());

        var shooterError = Math.abs(robotAngleRad - targetAngleRad);
        if (shooterError > 0.4) {
            //if off by more than ~22*, reset the error
            pid.reset();
        }
        if (  (  (Math.abs(shooter.getRpm() - lookup.getShooterSpeed()) < 300)
              && (Math.abs(aimer.getAngle() - (lookup.getAngle())) < 4)
              && (shooterError < 0.16)
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
        //intake.setRpm(250);
        //goodCounts++;
    }

    @Override
    public boolean isFinished() {
        return goodCounts > 40;
    }

    @Override
    public void end(boolean interrupted) {
        aimer.stop();
        intake.setPower(0);
        //keep shooter running in auto
        shooter.setRpm(3000);
        kicker.stop();
    }
}