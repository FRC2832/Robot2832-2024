package frc.robot.climber;

import edu.wpi.first.wpilibj2.command.Command;

public class HomeClimber extends Command {
    Inclinator climber;
    int leftTimer;
    int rightTimer;
    boolean isLeftHomed;
    boolean isRightHomed;
    final double kCurrentLimit = 1;
    final int kTimeoutThreshold = 10;
    final double kPower = -0.08;

    public HomeClimber(Inclinator climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        leftTimer = 0;
        rightTimer = 0;
        isLeftHomed = false;
        isRightHomed = false;
    }

    @Override
    public void execute() {
        if(!isLeftHomed){
            if(climber.getLeftLimit()) {
                leftTimer = kTimeoutThreshold;
            } else if(climber.getLeftCurrent() > kCurrentLimit){
                leftTimer++;
            }
            else {
                leftTimer = 0;
            }
            if(leftTimer >= kTimeoutThreshold){
                isLeftHomed = true;
                climber.setOffsetPower(0, true);
            } else {
                climber.setOffsetPower(kPower,true);
            }
        } 
        else if(!isRightHomed){
            if(climber.getRightLimit()) {
                rightTimer = kTimeoutThreshold;
            } else if(climber.getRightCurrent() > kCurrentLimit){
                rightTimer++;
            }
            else {
                rightTimer = 0;
            }
            if(rightTimer >= kTimeoutThreshold){
                isRightHomed = true;
                climber.setOffsetPower(0, false);
            } else {
                climber.setOffsetPower(kPower,false);
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
