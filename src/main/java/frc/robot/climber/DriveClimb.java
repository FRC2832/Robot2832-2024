package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls.IOperatorControls;

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
            climber.setPower(-.25);
        }
        // driving climber up
        else if(controller.IsClimbUpRequested()){
            climber.setPower(.25);
        }
        else if(controller.IsClimbLeftRequested()){
            climber.setOffsetPower(-0.25, true);
        }
        else if(controller.IsClimbRightRequested()){
            climber.setOffsetPower(-0.25, false);
        }
        else{
            climber.setPower(0);
        }
    }
}