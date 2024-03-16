package frc.robot.Controls;

public interface IDriveControls {
    double GetXDrivePct();
    double GetYDrivePct();
    double GetTurnPct();
    boolean IsFieldOrientedResetRequested();
    boolean IsIntakeRequested();
    boolean IsMaxSpeedRequested();
}
