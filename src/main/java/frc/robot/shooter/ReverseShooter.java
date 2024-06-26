package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.kicker.Kicker;

public class ReverseShooter extends Command {
    Shooter shooter;
    Kicker kicker;
    
    public ReverseShooter(Kicker kicker, Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter, kicker);
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
        return false;
    }

    @Override
    public void end(boolean interrupted){
        shooter.setRPM(0);
    }
}
    
