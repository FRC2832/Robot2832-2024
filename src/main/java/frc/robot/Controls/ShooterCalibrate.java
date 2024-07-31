package frc.robot.Controls;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;
import frc.robot.aimer.Aimer;

public class ShooterCalibrate extends Command{
    private Shooter shooter;
    private Kicker kicker;
    private Aimer aimer;
    private DoubleSubscriber shooterRpm, kickRpm, shooterAngle;

    public ShooterCalibrate(Shooter shooter, Kicker kicker, Aimer aimer) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.aimer = aimer;
        addRequirements(shooter, kicker, aimer);

        shooterRpm = UtilFunctions.getNtSub("/Cal/Shooter Cal RPM", 0.);
        kickRpm = UtilFunctions.getNtSub("/Cal/Kicker Cal RPM", 0.);
        shooterAngle = UtilFunctions.getNtSub("/Cal/Shooter Cal Angle", 0.);
    }
    
    @Override
    public void execute() {
        shooter.setRpm(shooterRpm.get());
        kicker.setRpm(kickRpm.get());
        aimer.goTo(shooterAngle.get());
    }
}
