package frc.robot.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TestIntake extends Command {
    private Intake intake;

    public TestIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
        SmartDashboard.putBoolean("Intake Running", false);
        SmartDashboard.putBoolean("Intake Inverted", false);
        SmartDashboard.putNumber("Intake Percent", 0);
    }

    @Override
    public void execute() {
        boolean isRunning = SmartDashboard.getBoolean("Intake Running", false);
        boolean inverted = SmartDashboard.getBoolean("Intake Inverted", false);
        intake.setIntake(isRunning, inverted);
    }
}
