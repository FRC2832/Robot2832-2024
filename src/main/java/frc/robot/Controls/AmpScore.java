package frc.robot.Controls;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.aimer.Aimer;
import frc.robot.amp.Amp;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;

public class AmpScore extends Command {
    Kicker kicker;
    Shooter shooter;
    Amp amp;
    Aimer aimer;
    DoubleSubscriber kickRpm;
    DoubleSubscriber shotRpm;
    DoubleSubscriber aimAngle;

    public AmpScore(Kicker kicker, Shooter shooter, Amp amp, Aimer aimer) {
        this.kicker = kicker;
        this.amp = amp;
        this.shooter = shooter;
        this.aimer = aimer;
        addRequirements(kicker, shooter, amp, aimer);

        kickRpm = UtilFunctions.getSettingSub("/Shooting/AmpKickRpm", 450.);
        shotRpm = UtilFunctions.getSettingSub("/Shooting/AmpShotRpm", 450.);
        aimAngle = UtilFunctions.getSettingSub("/Shooting/AmpShotAngle", 54.);
    }

    @Override
    public void execute() {
        amp.SetAmpDirection(true);
        kicker.setRpm(kickRpm.get());
        shooter.setRpm(shotRpm.get());
        aimer.goTo(aimAngle.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        amp.SetAmpDirection(false);
    }
}
