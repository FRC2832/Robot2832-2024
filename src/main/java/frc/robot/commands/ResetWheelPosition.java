package frc.robot.commands;

import org.livoniawarriors.swerve.SwerveDriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.livoniawarriors.odometry.Odometry;

public class ResetWheelPosition extends Command {
    
    private SwerveDriveTrain swerve;
    private Odometry odometry;
    
    public ResetWheelPosition(SwerveDriveTrain swerve, Odometry odometry) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.odometry = odometry;
    }

    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        swerve.resetSwervePositions();
        Pose2d pos = odometry.getPose();
        odometry.resetPose(pos);
    }
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
