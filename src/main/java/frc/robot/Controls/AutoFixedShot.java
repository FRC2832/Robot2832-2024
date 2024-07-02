package frc.robot.Controls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.aimer.Pneumatics;
import frc.robot.intake.Intake;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;

public class AutoFixedShot extends Command {
    private Intake intake;
    private Shooter shooter;
    private Kicker kicker;
    private Pneumatics aimer;
    private int goodCounts;

    private final double SHOT_RPM = 6000;
    private final double AIMER_ANGLE = 51;

    public AutoFixedShot(Intake intake, Shooter shooter, Kicker kicker, Pneumatics aimer) {
        this.intake = intake;
        this.shooter = shooter;
        this.kicker = kicker;
        this.aimer = aimer;
        addRequirements(intake, shooter, kicker, aimer);
    }

    @Override
    public void initialize() {
        goodCounts = 0;
    }

    @Override 
    public void execute() {
        shooter.setRpm(SHOT_RPM);
        kicker.setPower(1);
        aimer.goTo(AIMER_ANGLE);

        if((  shooter.getRpm() > 4800
          && Math.abs(aimer.getAngle() - AIMER_ANGLE) < 5)
          || goodCounts > 6
          )
        {
            intake.setRpm(250);
            //wait to start timeout until piece leaves starts leaving intake
            if(intake.isPieceDetected() == false) {
                goodCounts++;
            }  
        } else {
            intake.setPower(0);
        }
    }

    @Override
    public boolean isFinished() {
        return goodCounts > 40;
    }

    @Override
    public void end(boolean interrupted) {
        aimer.stop();
        intake.setPower(0);
        //keep shooter running in auto
        shooter.setRpm(6000);
        kicker.stop();
    }
}
