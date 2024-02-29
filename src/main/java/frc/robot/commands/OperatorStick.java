package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.interfaces.IOperatorControls;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriverFeedback;
import frc.robot.subsystems.Kicker;


public class OperatorStick extends Command {
    private Shooter shoot;
    private Kicker kick;
    private IOperatorControls cont;

    public OperatorStick(Shooter shoot, IOperatorControls cont, Kicker kick){
        this.shoot = shoot;
        this.cont = cont;
        this.kick = kick;
        addRequirements(shoot);
        addRequirements(kick);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Shooter RPM Command",4500);
        SmartDashboard.putNumber("Kicker RPM Command",4500);
    }

    @Override
    public void execute() {
        if(cont.IsSubShotRequested()){
            double shotRpm = SmartDashboard.getNumber("Shooter RPM Command", 4500);
            double kickRpm = SmartDashboard.getNumber("Kicker RPM Command", 4500);
            shoot.setRPM(shotRpm);
            kick.setRPM(kickRpm);

            if(Math.abs(shotRpm - shoot.getRPM()) < 50) {
                DriverFeedback.setColor(Color.kGreen);
            } else {
                DriverFeedback.setColor(Color.kYellow);
            }
        }
        else{
            shoot.setRPM(3500);
            kick.setRPM(0);
            DriverFeedback.setColor(Color.kBlack);
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