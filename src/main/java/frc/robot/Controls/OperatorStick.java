package frc.robot.Controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriverFeedback;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;
import frc.robot.aimer.Pneumatics;
import frc.robot.intake.Intake;


public class OperatorStick extends Command {
    private Shooter shoot;
    private Kicker kick;
    private Pneumatics aimer;
    private IOperatorControls cont;
    private Intake intake;

    public OperatorStick(Shooter shoot, IOperatorControls cont, Kicker kick, Pneumatics aimer, Intake intake){
        this.shoot = shoot;
        this.cont = cont;
        this.kick = kick;
        this.aimer = aimer;
        this.intake = intake;

        addRequirements(shoot);
        addRequirements(kick);

        SmartDashboard.putNumber("Shooter RPM Command",4500);
        SmartDashboard.putNumber("Kicker RPM Command",4500);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(cont.IsSubShotRequested()){
            double shotRpm = SmartDashboard.getNumber("Shooter RPM Command", 4500);
            double shotAngle = SmartDashboard.getNumber("Shooter Angle Command", 51);
            double kickRpm = SmartDashboard.getNumber("Kicker RPM Command", 4500);
            shoot.setRPM(shotRpm);
            kick.setRPM(kickRpm);
            aimer.goTo(shotAngle);

            if (  (Math.abs(shotRpm - shoot.getRPM()) < 75) 
               && (Math.abs(shotAngle - aimer.getAngle()) < 4)
               )
            {
                cont.rumbleController(RumbleType.kBothRumble, .5);
            } else {
                cont.rumbleController(RumbleType.kBothRumble, 0);
            }
        }
        else{
            shoot.setRPM(3500);
            if(intake.isPieceDetected()){
                kick.setRPM(3500);
            }
            else{
                kick.setRPM(0);
            }
            DriverFeedback.setColor(Color.kBlack);
            cont.rumbleController(RumbleType.kBothRumble, 0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shoot.setPower(0);
    }
}