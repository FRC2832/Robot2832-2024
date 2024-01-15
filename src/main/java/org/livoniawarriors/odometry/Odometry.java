package org.livoniawarriors.odometry;

import java.util.Optional;

import org.livoniawarriors.Logger;
import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.swerve.SwerveDriveTrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
    public static final double FIELD_LENGTH_METERS = 16.542;
    public static final double FIELD_WIDTH_METERS = 8.014;
    
    IGyroHardware hardware;
    double lastVisionTime;
    SwerveDriveOdometry odometry;
    SwerveDrivePoseEstimator poseEstimator;
    SwerveDriveTrain drive;
    Pose2d robotPose;
    Pose2d startPose;
    
    private Field2d field;
    private Translation2d[] swervePositions;
    private BooleanSubscriber resetPos;
    private BooleanSubscriber plotCorners;

    public Odometry() {
        super();
        hardware = new BlankGyro();
        robotPose = new Pose2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2, new Rotation2d());
        startPose = new Pose2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2, new Rotation2d());
        lastVisionTime = 0;

        field = new Field2d();
        resetPos = UtilFunctions.getNtSub("/Odometry/Reset Position", false);
        plotCorners = UtilFunctions.getSettingSub("/Odometry/Plot Swerve Corners", false);

        SmartDashboard.putData("Field", field);
        Logger.RegisterSensor("Gyro Yaw", this::getGyroAngle);
        Logger.RegisterSensor("Gyro Pitch", this::getGyroPitch);
        Logger.RegisterSensor("Gyro Roll", this::getGyroRoll);
        Logger.RegisterSensor("Gyro X Accel", hardware::getXAccel);
        Logger.RegisterSensor("Gyro Y Accel", hardware::getYAccel);
        Logger.RegisterSensor("Gyro Z Accel", hardware::getZAccel);
    }

    public void setSwerveDrive(SwerveDriveTrain drive) {
        this.drive = drive;
        swervePositions = drive.getCornerLocations();
        odometry = new SwerveDriveOdometry(
            drive.getKinematics(), 
            getGyroRotation(), 
            drive.getSwervePositions());
        poseEstimator = new SwerveDrivePoseEstimator(
            drive.getKinematics(),
            getGyroRotation(),
            drive.getSwervePositions(),
            startPose,
            //TODO: Make these calibratable
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9)
        );
    }

    public void setGyroHardware(IGyroHardware hardware) {
        this.hardware = hardware;
        hardware.updateHardware();
    }
   
    @Override
    public void periodic() {
        hardware.updateHardware();
        Rotation2d heading = getGyroRotation();

        if(drive != null) {
            SwerveModulePosition[] states = drive.getSwervePositions();
            robotPose = odometry.update(heading, states);
            poseEstimator.update(heading, states);
            field.setRobotPose(robotPose);
            field.getObject("Vision Pose").setPose(poseEstimator.getEstimatedPosition());

            Pose2d[] swervePoses;
            if(plotCorners.get()) {
                // Update the poses for the swerveModules. Note that the order of rotating the
                // position and then adding the translation matters
                swervePoses = new Pose2d[swervePositions.length];
                for (int i = 0; i < swervePositions.length; i++) {
                    Translation2d modulePositionFromChassis = swervePositions[i].rotateBy(heading).plus(robotPose.getTranslation());

                    // Module's heading is it's angle relative to the chassis heading
                    swervePoses[i] = new Pose2d(modulePositionFromChassis,
                        states[i].angle.plus(robotPose.getRotation()));
                }
            } else {
                swervePoses = new Pose2d[0];
            }
            field.getObject("Swerve Modules").setPoses(swervePoses);
        }

        //if the user requests a reset of position, do it
        if(resetPos.get()) {
            resetPose(flipAlliance(startPose));
            //turn off the request
            resetPos.getTopic().publish().set(false);
        }
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
     * always on the right corner of your alliance wall, so for 2023, the field
     * elements are at different coordinates for each alliance.
     * 
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    public static Pose2d flipAlliance(Pose2d poseToFlip) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return poseToFlip.relativeTo(new Pose2d(
                new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
                new Rotation2d(Math.PI)));
        } else {
            return poseToFlip;
        }
    }

    /**
     * Supplier that determines if paths should be flipped to the other side of the field. This will maintain a global blue alliance origin.
     * @return If we need to flip
     */
    public boolean shouldFlipAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return true;
        } else {
            return false;
        }
    }

    public void setStartingPose(Pose2d pose) {
        startPose = flipAlliance(pose);
        resetPose(startPose);
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getGyroRotation(), drive.getSwervePositions(), pose);
    }

    public void resetHeading() {
        //reset the robot back to it's spot, just facing forward now
        Pose2d pose = new Pose2d(robotPose.getTranslation(),Rotation2d.fromDegrees(0));
        odometry.resetPosition(getGyroRotation(), drive.getSwervePositions(), pose);
    }

    public Pose2d getPose() {
        return robotPose;
    }

    public Rotation2d getHeading() {
        return robotPose.getRotation();
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    public double getGyroAngle() {
        return hardware.getGyroAngle();
    }

    public double getGyroPitch() {
        return hardware.getPitchAngle();
    }

    public double getGyroRoll() {
        return hardware.getRollAngle();
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDeviation) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDeviation);
    }
}
