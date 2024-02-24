package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import org.livoniawarriors.swerve.MoveWheels;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Pneumatics;

// Plot RPM values
// PID F RPMnn   
// Put RPM on smartdashboard
// Try a spin
  
public class Diagnostics extends Command{
    private Intake intake;
    private Shooter shooter;
    private Pneumatics pneumatics;

    public Diagnostics(Intake intake, Shooter shooter, Pneumatics pneumatics) {
        this.intake = intake;
        this.shooter = shooter;
        this.pneumatics = pneumatics;
        addRequirements(intake, shooter, pneumatics);     
    }
    

    @Override
    public void execute() {
        pneumatics.diagnosticCheck();
    
        // Intake motor rpm
        double intakeRpm = 500.0;
        intake.setRpm(intakeRpm);

        // Call swerve test commands (driveWheelsStraight & driveWheelsDiamond) from "MoveWheels"
        MoveWheels.DriveWheelsStraight();
        MoveWheels.DriveWheelsDiamond();

        // Medium shooter speed test
        double shooterRpm = 500.0;
        shooter.setRPM(shooterRpm);
    }
}
