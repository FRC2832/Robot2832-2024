package frc.robot.Controls;
import frc.robot.intake.Intake;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;
import frc.robot.aimer.Pneumatics;
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
    Pneumatics pneumatic;
    Kicker kicker;
    Odometry odometry;
    Intake intake;
    Double tagY;
    SwerveDriveTrain swerveDrive;
    PIDController pid;
    int goodCounts;

    public Autoshot(Shooter shooter, Pneumatics pneumatic, Kicker kicker, Odometry odometry, Intake intake, SwerveDriveTrain swerveDrive) {
        this.shooter = shooter;
        this.pneumatic = pneumatic;
        this.kicker = kicker;
        this.odometry = odometry;
        this.intake = intake;
        this.tagY = 218.42 * 0.0254;
        this.swerveDrive = swerveDrive;
        this.pid = new PIDController(.28/Math.PI, .45, 0);
        addRequirements(shooter, pneumatic, kicker, intake, swerveDrive);
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
        
        //calculate robot distance
        var distance = UtilFunctions.getDistance(new Pose2d(tagX, tagY, null), robotPose);
        AutoShotLookup lookup = shooter.estimate(distance);

        var shooterError = Math.abs(robotAngleRad - targetAngleRad);
        if (shooterError > 0.4) {
            //if off by more than ~22*, reset the error
            pid.reset();
        }

        SmartDashboard.putNumber("Auto Drive Aim Error", shooterError);
        SmartDashboard.putNumber("Auto Shoot RPM Error", Math.abs(shooter.getRPM() - lookup.getShooterSpeed()));
        SmartDashboard.putNumber("Auto Aimer Aim Error", Math.abs(pneumatic.getAngle() - (lookup.getAngle())));

        if (goodCounts > 10) {
            //if we have see the target enough send it
            intake.setRpm(250);
            goodCounts++;
        }
        else if (  (  (Math.abs(shooter.getRPM() - lookup.getShooterSpeed()) < 300)
                   && (Math.abs(pneumatic.getAngle() - (lookup.getAngle())) < 3)
                   && (shooterError < 0.08)
                   )
                )
        {
            //we are starting to see the target, wait
            intake.setPower(0);
            goodCounts++;
            swerveDrive.SwerveDrive(0,0,0);
        }
        else {
            //target missing, reset counts
            intake.setPower(0);
            shooter.setRPM(lookup.getShooterSpeed());
            pneumatic.goToSmooth(lookup.getAngle());
            kicker.setRPM(lookup.getKickerSpeed());
            swerveDrive.SwerveDrive(0,0,angleCommand);
            goodCounts = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return goodCounts > 60;
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