package frc.robot.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
  
public class TestSpinShot extends Command{
    private Shooter shooter;

    public TestSpinShot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        SmartDashboard.putNumber("Spin Difference", 0);
        SmartDashboard.putBoolean("Spin Direction (right?)", true);
    }
    
    @Override
    public void execute() {
        var spinDiff = SmartDashboard.getNumber("Spin Difference", 0);
        var spinDirection = SmartDashboard.getBoolean("Spin Direction (right?)", true);
        shooter.spinShot(spinDiff, spinDirection);
    }
}
