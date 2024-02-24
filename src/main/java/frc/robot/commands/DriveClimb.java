package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.interfaces.IOperatorControls;
import frc.robot.subsystems.Inclinator;

public class DriveClimb extends Command {
    IOperatorControls controller;
    Inclinator climber;
    public DriveClimb(Inclinator climber, IOperatorControls controller) {
        this.controller = controller;
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        // driving climber down
        if(controller.IsClimbDownRequested()){
            climber.setPower(-0.75);
        }
        // driving climber up
        else if(controller.IsClimbUpRequested()){
            climber.setPower(0.75);
        }
        else if(controller.IsClimbLeftRequested()){
            climber.setOffsetPower(0.5, false);
        }
        else if(controller.IsClimbRightRequested()){
            climber.setOffsetPower(0.5, true);
        }
        else{
            climber.setPower(0);
        }
    }
}
