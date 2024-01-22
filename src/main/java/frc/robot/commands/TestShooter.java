package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

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
