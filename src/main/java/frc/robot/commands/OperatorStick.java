package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.interfaces.IOperatorControls;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.kicker;


public class OperatorStick extends Command {
    private Shooter shoot;
    private kicker kick;
    private IOperatorControls cont;

    public OperatorStick(Shooter shoot, IOperatorControls cont, kicker kick){
        this.shoot = shoot;
        this.cont = cont;
        this.kick = kick;
        addRequirements(shoot);
        addRequirements(kick);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Shooter RPM Command",3000);
        SmartDashboard.putNumber("Kicker RPM Command",4000);
    }

    @Override
    public void execute() {
        if(cont.IsSubShotRequested()){
            shoot.setRPM(SmartDashboard.getNumber("Shooter RPM Command", 3000));
            kick.setRPM(SmartDashboard.getNumber("Kicker RPM Command", 4000));
        }
        else{
            shoot.setPower(0);
            kick.setPower(0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}