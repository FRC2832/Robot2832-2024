package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class PitIntake extends Command {
    private Intake intake;

    public PitIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setRpm(300);
    }

    @Override
    public void end(boolean interrupted) {
        //we want to use the set power here to ramp down the motor, not PID control
        intake.setPower(0);
    }
}
