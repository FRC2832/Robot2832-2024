package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveIntake extends Command {
    private Intake intake;
    private boolean stop;

    public DriveIntake(Intake intake, boolean stop) {
        this.intake = intake;
        this.stop = stop;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setIntake(true, false);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntake(false, false);
    }
}
