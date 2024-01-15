package org.livoniawarriors.odometry;

import org.livoniawarriors.swerve.SwerveDriveTrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;

public class SimSwerveGyro implements IGyroHardware {
    private SwerveDriveTrain swerve;
    private double chassisAngle;
    private double accelX, accelY;
    private ChassisSpeeds lastSpeeds;

    public SimSwerveGyro(SwerveDriveTrain swerve) {
        this.swerve = swerve;
        chassisAngle = 0;
        accelX = 0;
        accelY = 0;
        lastSpeeds = new ChassisSpeeds();
    }

    @Override
    public void updateHardware() {
        ChassisSpeeds speeds = swerve.getKinematics().toChassisSpeeds(swerve.getSwerveStates());
        chassisAngle += Math.toDegrees(speeds.omegaRadiansPerSecond * TimedRobot.kDefaultPeriod);

        accelX = (speeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond) / TimedRobot.kDefaultPeriod;
        accelY = (speeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond) / TimedRobot.kDefaultPeriod;
        lastSpeeds = speeds;
    }

    @Override
    public double getGyroAngle() {
        return chassisAngle;
    }

    @Override
    public double getPitchAngle() {
        return 0;
    }

    @Override
    public double getRollAngle() {
        return 0;
    }

    @Override
    public double getXAccel() {
        return accelX;
    }

    @Override
    public double getYAccel() {
        return accelY;
    }

    @Override
    public double getZAccel() {
        return 0;
    }
    
}
