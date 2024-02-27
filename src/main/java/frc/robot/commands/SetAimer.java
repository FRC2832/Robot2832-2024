package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pneumatics;

public class SetAimer extends Command {
    private Pneumatics aimer;
    private double angle;

    public SetAimer(Pneumatics aimer, double angle) {
        this.aimer = aimer;
        this.angle = angle;
        addRequirements(aimer);
    }
    
    @Override
    public void execute() {
        aimer.goTo(angle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(aimer.getAngle() - angle) < 5;
    }
}
