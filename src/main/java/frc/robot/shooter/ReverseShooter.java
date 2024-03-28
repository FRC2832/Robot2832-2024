package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ReverseShooter extends Command {
    Shooter shooter;
    public ReverseShooter(Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        shooter.setRPM(-3000);
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted){
        shooter.setRPM(0);
    }
}
    
