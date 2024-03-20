package frc.robot.Controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public interface IOperatorControls {
    boolean AutoSubAimRequested();
    double GetManualSubAim();
    boolean IsSubShotRequested();
    boolean IsCenterFieldShotRequested();
    boolean IsPillarShotRequested();
    boolean IsClimbLeftRequested();
    boolean IsClimbRightRequested();
    boolean IsClimbUpRequested();
    boolean IsClimbDownRequested();
    boolean IsClimbRequested();
    boolean IsIntakeRequested();
    boolean IsIntakeDownRequested();
    boolean IsAmpToggled();
    double GetTurnPct();
    void rumbleController(RumbleType type, double value);
}