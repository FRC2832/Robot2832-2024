package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.odometry.Odometry;

public class Autoshot extends Command {
    Shooter shooter;
    Pneumatics pneumatic;
    Kicker kicker;
    Odometry odometry;
    Double tagY;

    public Autoshot(Shooter shooter, Pneumatics pneumatic, Kicker kicker, Odometry odometry) {
        this.shooter = shooter;
        this.pneumatic = pneumatic;
        this.kicker = kicker;
        this.odometry = odometry;
        this.tagY = 218.42 * 0.0254;
    }

    //TODO: Physics based approach TBD
    public double[] calculate() {
        double [] result = {0.0,0.0};
        return result;
    }
    
    @Override 
    public void execute(){
        var tagX = (UtilFunctions.getAlliance() == Alliance.Red ? 652.73 : -1.5) * 0.0254;

        var robotPose = odometry.getPose();
        
        var distance = UtilFunctions.getDistance(new Pose2d(tagX, tagY, null), robotPose); //TODO: Need to handle rotation

        double[] values = shooter.estimate(distance);

        shooter.setRPM(values[0]);
        pneumatic.goTo(values[1]);
    }
}