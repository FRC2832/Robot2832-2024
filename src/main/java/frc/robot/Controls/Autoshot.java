package frc.robot.Controls;
import frc.robot.intake.Intake;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;
import frc.robot.aimer.Pneumatics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import org.livoniawarriors.AutoShotLookup;
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

    public Autoshot(Shooter shooter, Pneumatics pneumatic, Kicker kicker, Odometry odometry, Intake intake) {
        this.shooter = shooter;
        this.pneumatic = pneumatic;
        this.kicker = kicker;
        this.odometry = odometry;
        this.intake = intake;
        this.tagY = 218.42 * 0.0254;
        addRequirements(shooter, pneumatic, kicker, intake);
    }

    @Override
    public void initialize() {
        goodCounts = 0;
    }

    @Override 
    public void execute(){
        var tagX = (UtilFunctions.getAlliance() == Alliance.Red ? 652.73 : -1.5) * 0.0254;

        var robotPose = odometry.getPose();
        
        var distance = UtilFunctions.getDistance(new Pose2d(tagX, tagY, null), robotPose); //TODO: Need to handle rotation

        AutoShotLookup lookup = shooter.estimate(distance);

        shooter.setRPM(lookup.getShooterSpeed());
        pneumatic.goTo(lookup.getAngle());
        kicker.setRPM(lookup.getKickerSpeed());
        if (  (  (Math.abs(shooter.getRPM() - lookup.getShooterSpeed()) < 75)
              && (Math.abs(pneumatic.getAngle() - lookup.getAngle()) < 4)
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