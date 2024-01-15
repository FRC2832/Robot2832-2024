package org.livoniawarriors.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveWheels extends Command {
    private SwerveDriveTrain drive;
    private SwerveModuleState[] states;

    public MoveWheels(SwerveDriveTrain drive, SwerveModuleState[] states) {
        this.drive = drive;
        this.states = states;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drive.setWheelCommand(states);
    }

    @Override
    public boolean isFinished() {
        SwerveModulePosition[] positions = drive.getSwervePositions();
        boolean result = true;
        for(int i=0; i<states.length; i++) {
            double angle = positions[i].angle.minus(states[i].angle).getDegrees();
            angle = MathUtil.inputModulus(angle, -180, 180);
            if (Math.abs(angle) > 3 || states[i].speedMetersPerSecond > drive.getMinSpeed()) {
                //never leave if this requests speed
                result = false;
            }
        }
        return result; 
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopWheels();
    }

    public static SwerveModuleState[] WheelsStraight() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        states[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        states[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        states[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        return states;
    }

    public static SwerveModuleState[] WheelsDiamond() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(315));
        states[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(225));
        states[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        states[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(135));
        return states;
    }

    public static SwerveModuleState[] WheelsCrossed() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        states[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(315));
        states[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(315));
        states[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        return states;
    }

    public static SwerveModuleState[] DriveWheelsStraight() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState(1, Rotation2d.fromDegrees(0));
        states[1] = new SwerveModuleState(1, Rotation2d.fromDegrees(0));
        states[2] = new SwerveModuleState(1, Rotation2d.fromDegrees(0));
        states[3] = new SwerveModuleState(1, Rotation2d.fromDegrees(0));
        return states;
    }

    public static SwerveModuleState[] DriveWheelsDiamond() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = new SwerveModuleState(1, Rotation2d.fromDegrees(315));
        states[1] = new SwerveModuleState(1, Rotation2d.fromDegrees(225));
        states[2] = new SwerveModuleState(1, Rotation2d.fromDegrees(45));
        states[3] = new SwerveModuleState(1, Rotation2d.fromDegrees(135));
        return states;
    }
}
