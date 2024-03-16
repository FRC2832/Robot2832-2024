package frc.robot.Controls;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

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
        return UtilFunctions.deadband(cont.getLeftY(), dead);
    }

    @Override
    public boolean IsSubShotRequested() {
        return cont.getYButton();
    }

    @Override
    public boolean IsClimbLeftRequested() {
        return cont.getPOV()==270;
    }

    @Override
    public boolean IsClimbRightRequested() {
        return cont.getPOV()==90;
    }

    @Override
    public boolean IsClimbUpRequested() {
        int pov = cont.getPOV();
        return pov == 0 || pov == 45 || pov == 315;
    }

    @Override
    public boolean IsClimbDownRequested() {
        int pov = cont.getPOV();
        return pov == 180 || pov == 135 || pov == 225;
    }

    @Override
    public boolean IsClimbRequested() {
        return cont.getPOV()>=0;
    }

    @Override
    public boolean IsIntakeRequested() {
        return cont.getRightTriggerAxis()>.5;
    }

    @Override
    public double GetTurnPct() {
        var dead = deadband.get();
        return UtilFunctions.deadband(-cont.getLeftX(), dead);
    }


    @Override
    public boolean IsIntakeDownRequested() {
        return cont.getLeftTriggerAxis()>.5;
    }
    
    @Override
    public void rumbleController(RumbleType type, double value) {
        cont.setRumble(type, value);
    }

    @Override
    public boolean IsCenterFieldShotRequested() {
        return cont.getBButtonPressed();
    }

    @Override
    public boolean IsPillarShotRequested() {
        return cont.getXButtonPressed();
    }
}
