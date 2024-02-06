package frc.robot.interfaces;

public interface IOperatorControls {
    boolean AutoSubAimRequested();
    double GetManualSubAim();
    boolean IsSubShotRequested();
    boolean IsClimbLeftRequested();
    boolean IsClimbRightRequested();
    boolean IsClimbUpRequested();
    boolean IsClimbDownRequested();
    boolean IsManualShotRequested();
    double GetTurnPct();
}