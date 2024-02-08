package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.interfaces.IOperatorControls;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class OperatorStick extends Command {
    private Shooter shoot;
    private Intake intake;
    private IOperatorControls cont;
    private boolean isShootRequested;

    public OperatorStick(Shooter shoot, IOperatorControls cont, Intake intake){
        this.shoot = shoot;
        this.cont = cont;
        this.intake = intake;
        addRequirements(shoot);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Intake Command",0);
        SmartDashboard.putNumber("Shooter Command",0);
    }

    @Override
    public void execute() {
        if(cont.IsSubShotRequested()){
            shoot.setPower(SmartDashboard.getNumber("Shooter Command", 0));
            intake.setPower(SmartDashboard.getNumber("Intake Command", 0));
        }
        else{
            shoot.setPower(0);
            intake.setPower(0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}