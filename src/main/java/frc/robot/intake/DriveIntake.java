package frc.robot.intake;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveIntake extends Command {
    private Intake intake;
    private boolean stop;
    private boolean invert;
    DoubleSubscriber intakeSpeed;

    public DriveIntake(Intake intake, boolean stop) {
        this(intake, stop, false);
    }

    public DriveIntake(Intake intake, boolean stop, boolean invert) {
        this.intake = intake;
        this.stop = stop;
        this.invert = invert;
        addRequirements(intake);
        intakeSpeed = UtilFunctions.getSettingSub("/DriveIntake/Intake Speed", 150);
    }

    @Override
    public void execute() {
        double dir = 1;
        if(invert) dir = -1;
        if(stop && intake.isPieceDetected()) {
            intake.setPower(0);
        } else {
            intake.setRpm(dir * intakeSpeed.get());
        }
    }

    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous() && intake.isPieceDetected()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        //we want to use the set power here to ramp down the motor, not PID control
        intake.setPower(0);
    }
}
