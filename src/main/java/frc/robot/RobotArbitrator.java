package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.aimer.Pneumatics;
import frc.robot.amp.Amp;
import frc.robot.climber.Inclinator;

public class RobotArbitrator extends SubsystemBase {
    private Amp amp;
    private Inclinator climber;
    private Pneumatics aimer;
    private final double MIN_CLIMBER_HEIGHT = 26.5;
    private final double MOVE_AIMER_HEIGHT = 37; 
    private final double STOW_AIMER_ANGLE = 35;

    public RobotArbitrator(Amp amp, Inclinator climber, Pneumatics aimer) {
        super();
        this.amp = amp;
        this.climber = climber;
        this.aimer = aimer;
    }

    @Override
    public void periodic() {
        double climbLeft = climber.getLeftRequest();
        double climbRight = climber.getRightRequest();
        boolean isHoming = false;

        if(climber.getCurrentCommand() != null) {
            isHoming = climber.getCurrentCommand().getName().equals("HomeClimber");
        }

        if(amp.IsRaised()) {
            //amp is up, stop climber
            climbLeft = 0;
            climbRight = 0;
        }
        
        if(climbLeft < -0.1 && !isHoming) {
            double height = climber.getLeftHeight();
            if (height < MOVE_AIMER_HEIGHT) {
                aimer.goTo(STOW_AIMER_ANGLE);
            }
            if(height < MIN_CLIMBER_HEIGHT) {
                climbLeft = 0;
            }
        }

        if(climbRight < -0.1 && !isHoming) {
            double height = climber.getRightHeight();
            if (height < MOVE_AIMER_HEIGHT) {
                aimer.goTo(STOW_AIMER_ANGLE);
            }
            if(height < MIN_CLIMBER_HEIGHT) {
                climbRight = 0;
            }
        }

        climber.setArbRequest(climbLeft, climbRight);
    }
}
