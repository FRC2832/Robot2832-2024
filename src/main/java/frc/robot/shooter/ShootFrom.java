package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls.AutoShotLookup;
import frc.robot.aimer.Pneumatics;
import frc.robot.intake.Intake;
import frc.robot.kicker.Kicker;

public class ShootFrom extends Command {
    private Shooter shooter;
    private Pneumatics pneumatics;
    private Kicker kicker;
    private Intake intake;
    private AutoShotLookup shootData;
    private boolean shotCalled;
    private boolean IsCenterFieldShot;
    private int goodCounts;

    public ShootFrom(Shooter shooter, Pneumatics pneumatics, Kicker kicker, Intake intake, boolean IsCenterFieldShot) {
        this.shooter = shooter;
        this.pneumatics = pneumatics;
        this.kicker = kicker;
        this.intake = intake;
        this.IsCenterFieldShot = IsCenterFieldShot;
        addRequirements(shooter, pneumatics, kicker, intake);
    }

    @Override
    public void initialize() {
        shootData = new AutoShotLookup(0, 0, 0);
        goodCounts = 0;
        shotCalled = false;
    }
    
    @Override
    public void execute() {
        if(!shotCalled) {
            shootData.getPrefs(IsCenterFieldShot);
            handleShot(shootData);
            shotCalled = true;
        }
        if (shotCalled && !intake.isPieceDetected()) {
            goodCounts++;
        }
    }

    public void handleShot(AutoShotLookup data) {
        shooter.setRPM(data.getShooterSpeed());
        kicker.setRPM(data.getKickerSpeed());
        pneumatics.goTo(data.getAngle());
        intake.setRpm(250);
    }

    @Override
    public boolean isFinished() {
        return goodCounts > 40;
    }

    @Override
    public void end(boolean interrupted) {
        pneumatics.stop();
        intake.setPower(0);
        //keep shooter running in auto
        shooter.setRPM(3000);
        kicker.stop();
    }
}