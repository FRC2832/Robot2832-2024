package frc.robot.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//Plot RPM values
// PID F RPMnn   
// Put RPM on smartdashboard
// Try a spin
  
public class TestShooter extends Command{
    private Shooter shooter;

    public TestShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        SmartDashboard.putNumber("Shooter Command", 0);
    }
    
    @Override
    public void execute() {
        var power = SmartDashboard.getNumber("Shooter Command", 0);
        shooter.setPower(power);
    }
}