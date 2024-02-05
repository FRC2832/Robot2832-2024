package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Inclinator;

public class DriveClimb extends Command {
    XboxController controller;
    Inclinator climber;
    public DriveClimb(Inclinator climber) {
        controller = new XboxController(2);
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        int pov = controller.getPOV();
        // driving climber down
        if(pov == 180 || pov == 225 || pov == 135){
            climber.setPower(-0.75);
        }
        // driving climber up
        else if(pov == 0 || pov == 315 || pov == 45){
            climber.setPower(0.75);
        }
        else if(pov == 90){
            climber.setOffsetPower(0.5, false);
        }
        else if(pov == 270){
            climber.setOffsetPower(0.5, true);
        }
        else{
            climber.setPower(0);
        }
        SmartDashboard.putNumber("Pov", controller.getPOV());
    }
}
