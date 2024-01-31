package frc.robot.interfaces;

public interface IOperatorControls {
    
    boolean autoSubAimRequested();
    double getManualSubAim();
    boolean IsSubShotRequested();
    boolean IsClimbLeftRequested();
    boolean IsClimbRightRequested();
    boolean IsClimbUpRequested();
    boolean IsClimbDownRequested();
    
}