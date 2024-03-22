package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class StartShooter extends Command{
    private Shooter shooter;

    public StartShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute(){
        shooter.setRPM(6000);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
