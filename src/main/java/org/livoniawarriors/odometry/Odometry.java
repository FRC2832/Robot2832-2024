package org.livoniawarriors.odometry;

import org.livoniawarriors.Logger;
import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.swerve.SwerveDriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
    public static final double FIELD_LENGTH_METERS = 16.542;
    public static final double FIELD_WIDTH_METERS = 8.014;
    
    IGyroHardware hardware;
    double lastVisionTime;
    SwerveDrivePoseEstimator poseEstimator;
    SwerveDriveTrain drive;
    Pose2d robotPose;
    Pose2d startPose;
    int loopsTagsSeen;
    
    private Field2d field;
    private Translation2d[] swervePositions;
    private BooleanSubscriber resetPos;
    private BooleanSubscriber plotCorners;
    private BooleanSubscriber useVision;
    private DoublePublisher visionOffset, pathRotOffset;
    private DoubleArraySubscriber pathTarget, pathCurrent;
    private IntegerPublisher visionFrameCount;
    private long frameCount;
    private boolean startPosReceived;

    public Odometry() {
        super();
        hardware = new BlankGyro();
        robotPose = new Pose2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2, new Rotation2d());
        startPose = new Pose2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2, new Rotation2d());
        lastVisionTime = 0;
        frameCount = 0;
        loopsTagsSeen = 0;
        startPosReceived = false;

        field = new Field2d();
        resetPos = UtilFunctions.getNtSub("/Odometry/Reset Position", false);
        plotCorners = UtilFunctions.getSettingSub("/Odometry/Plot Swerve Corners", false);
        useVision = UtilFunctions.getSettingSub("/Odometry/Use Vision", false);
        visionOffset = UtilFunctions.getNtPub("/Odometry/Vision Error", 0.);
        visionFrameCount = UtilFunctions.getNtPub("/Odometry/VisionFrames", 0);
        pathCurrent = UtilFunctions.getNtSub("/PathPlanner/currentPose", new double[3]);
        pathTarget = UtilFunctions.getNtSub("/PathPlanner/targetPose", new double[3]);
        pathRotOffset = UtilFunctions.getNtPub("/PathPlanner/rotInaccuracy", 0.);

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
        poseEstimator = new SwerveDrivePoseEstimator(
            drive.getKinematics(),
            getGyroRotation(),
            drive.getSwervePositions(),
            startPose,
            //TODO: Make these calibratable
            //X, Y , Angle 
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(9, 9, 9)
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

        //fill out the rotation error for pathplanner
        var current = pathCurrent.get();
        var target = pathTarget.get();
        pathRotOffset.set(MathUtil.inputModulus(current[2] - target[2],-Math.PI, Math.PI));

        if(drive != null) {
            SwerveModulePosition[] states = drive.getSwervePositions();
            robotPose = poseEstimator.update(heading, states);
            field.setRobotPose(robotPose);
            Pose2d speaker;
            if(UtilFunctions.getAlliance() == Alliance.Red) {
                speaker = new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), new Rotation2d());
            } else {
                speaker = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), new Rotation2d());
            }
            SmartDashboard.putNumber("Robot to Speaker", UtilFunctions.getDistance(robotPose, speaker));
            
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
        if (UtilFunctions.getAlliance() == Alliance.Red) {
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
        if (UtilFunctions.getAlliance() == Alliance.Red) {
            return true;
        } else {
            return false;
        }
    }

    public void setStartingPose(Pose2d pose) {
        startPose = pose;
        resetPose(startPose);
        startPosReceived = true;
    }

    public Pose2d getStartingPose() {
        return startPose;
    }

    public boolean IsStartSet() {
        return startPosReceived;
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation(), drive.getSwervePositions(), pose);
        robotPose = pose;
    }

    public void resetHeading() {
        //reset the robot back to it's spot, just facing forward now
        Pose2d pose = new Pose2d(robotPose.getTranslation(),Rotation2d.fromDegrees(0));
        poseEstimator.resetPosition(getGyroRotation(), drive.getSwervePositions(), pose);
        robotPose = pose;
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
        if(useVision.get()) {
            poseEstimator.addVisionMeasurement(pose, timestamp);
        }
        visionOffset.set(UtilFunctions.getDistance(pose, robotPose));
        visionFrameCount.set(frameCount++);
        loopsTagsSeen++;
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDeviation) {
        if(useVision.get()) {
            poseEstimator.addVisionMeasurement(pose, timestamp, stdDeviation);
            loopsTagsSeen++;
        }
        visionOffset.set(UtilFunctions.getDistance(pose, robotPose));
        visionFrameCount.set(frameCount++);
        loopsTagsSeen++;
    }

    /**
     * Returns how many frames that we have seen an April Tag.
     * @return
     */
    public int getLoopsTagSeen() {
        return loopsTagsSeen;
    }
}
