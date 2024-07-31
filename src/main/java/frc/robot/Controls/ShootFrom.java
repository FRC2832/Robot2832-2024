package frc.robot.Controls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.aimer.Aimer;
import frc.robot.intake.Intake;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;

public class ShootFrom extends Command {
    private Shooter shooter;
    private Aimer aimer;
    private Kicker kicker;
    private Intake intake;
    private AutoShotLookup shootData;
    private boolean shotCalled;
    private boolean IsCenterFieldShot;

    public ShootFrom(Shooter shooter, Aimer aimer, Kicker kicker, Intake intake, boolean IsCenterFieldShot) {
        this.shooter = shooter;
        this.aimer = aimer;
        this.kicker = kicker;
        this.intake = intake;
        this.IsCenterFieldShot = IsCenterFieldShot;
        addRequirements(shooter, aimer, kicker, intake);
    }

    @Override
    public void initialize() {
        shootData = new AutoShotLookup(0, 0, 0);
        shotCalled = false;
    }
    
    @Override
    public void execute() {
        if(!shotCalled) {
            shootData.getPrefs(IsCenterFieldShot);
            handleShot(shootData);
            shotCalled = true;
        }
    }

    public void handleShot(AutoShotLookup data) {
        shooter.setRpm(data.getShooterSpeed());
        kicker.setRpm(data.getKickerSpeed());
        //pneumatics.goTo(data.getAngle());
        aimer.goToSmooth(data.getAngle());
        intake.setRpm(250);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        aimer.stop();
        intake.setPower(0);
        //keep shooter running in auto
        shooter.setRpm(3000);
        kicker.stop();
    }
}