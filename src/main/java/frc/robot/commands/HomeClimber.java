package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Inclinator;

public class HomeClimber extends Command {
    Inclinator climber;
    int leftTimer;
    int rightTimer;
    boolean isLeftHomed;
    boolean isRightHomed;
    final double kCurrentLimit = 10;
    final int kTimeoutThreshold = 10;
    final double kPower = -0.2;

    public HomeClimber(Inclinator climber) {
        this.climber = climber;
        addRequirements(climber);
        leftTimer = 0;
        rightTimer = 0;
        isLeftHomed = false;
        isRightHomed = false;
    }

    @Override
    public void execute() {
        if(!isLeftHomed){
            climber.setOffsetPower(kPower,true);
            if(climber.getLeftCurrent() > kCurrentLimit){
                leftTimer++;
            }
            else {
                leftTimer = 0;
            }
            if(leftTimer > kTimeoutThreshold){
                isLeftHomed = true;
                climber.setOffsetPower(0, true);
            }
        } 
        else if(!isRightHomed){
            climber.setOffsetPower(kPower,false);
            if(climber.getRightCurrent() > kCurrentLimit){
                rightTimer++;
            }
            else {
                rightTimer = 0;
            }
            if(rightTimer > kTimeoutThreshold){
                isRightHomed = true;
                climber.setOffsetPower(0, false);
            }
        }
        else{
            // both homed, don't need to change anything ♡☆♡
        }
    }

    @Override
    public boolean isFinished() {
        return isLeftHomed && isRightHomed;
    }
}
