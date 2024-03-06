package frc.robot.aimer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls.IOperatorControls;

public class DriveAimer extends Command {
    private IOperatorControls controls;
    private Pneumatics pneumatics;

    public DriveAimer(IOperatorControls controls, Pneumatics pneumatics) {
        this.controls = controls;
        this.pneumatics = pneumatics;
        addRequirements(pneumatics);
    }

    @Override
    public void execute() {
        double command = controls.GetManualSubAim();
        if(command > 0.2) {
            pneumatics.driveDown();
        } else if (command < -0.2 && pneumatics.getAngle() < 50) {
            pneumatics.driveUp();
        } else {
            pneumatics.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.stop();
    }
}
