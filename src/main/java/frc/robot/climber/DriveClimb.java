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
            climber.setPower(-.65);
        }
        // driving climber up
        else if(controller.IsClimbUpRequested()){
            climber.setPower(.65);
        }
        else if(controller.IsClimbLeftRequested()){
            climber.setOffsetPower(-0.65, false);
        }
        else if(controller.IsClimbRightRequested()){
            climber.setOffsetPower(-0.65, true);
        }
        else{
            climber.setPower(0);
        }
    }
}
