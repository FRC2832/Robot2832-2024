package frc.robot.Controls;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.interfaces.IOperatorControls;

public class OperatorControls implements IOperatorControls{
    
    private XboxController cont;
    private DoubleSubscriber deadband;

    public OperatorControls(){
        cont = new XboxController(2);
        deadband = UtilFunctions.getSettingSub("DriveXbox/Deadband", 0.13);
    }


    @Override
    public boolean AutoSubAimRequested() {
        return cont.getAButton();
    }

    @Override
    public double GetManualSubAim() {
        var dead = deadband.get();
        return UtilFunctions.deadband(-cont.getRightY(), dead);
    }

    @Override
    public boolean IsSubShotRequested() {
        return cont.getYButton();
    }

    @Override
    public boolean IsClimbLeftRequested() {
        return cont.getPOV()==180;
    }

    @Override
    public boolean IsClimbRightRequested() {
        return cont.getPOV()==0;
    }

    @Override
    public boolean IsClimbUpRequested() {
        return cont.getPOV()==90;
    }

    @Override
    public boolean IsClimbDownRequested() {
        return cont.getPOV()==270;
    }

    @Override
    public boolean IsManualShotRequested() {
        return cont.getLeftTriggerAxis()>.5;
    }

    @Override
    public double GetTurnPct() {
        var dead = deadband.get();
        return UtilFunctions.deadband(-cont.getLeftX(), dead);
    }
    
}
