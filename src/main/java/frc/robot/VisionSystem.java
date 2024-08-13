package frc.robot;

import java.io.IOException;
import java.util.LinkedList;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.livoniawarriors.UtilFunctions;
import org.livoniawarriors.odometry.Odometry;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.AddVisionParams;

public class VisionSystem extends SubsystemBase {
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator frontCamEstimator;
    private VisionSystemSim visionSim;
    private boolean simInit;

    private PhotonCamera frontCam;
    private Transform3d frontCamPos;
    private double lastTargetTimestamp;
    private static final double APRILTAG_POSE_AMBIGUITY_THRESHOLD = 0.25;
    
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDeviations = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDeviations = VecBuilder.fill(0.5, 0.5, 1);
    public static final double kMaxVisionDistance = 4;
    private DoubleSubscriber visionStdDev;

    private IntegerSubscriber heartbeatSub;
    private long heartbeatLast;
    private int heartbeatMisses;
    private CameraData cameras[];
    private static Supplier<Pose2d> poseSupplier;
    private Consumer<AddVisionParams> addVisionMeasurement;

    public static enum FieldLocation {
        Speaker,
        Amp
    }

    public VisionSystem(Supplier<Pose2d> poseSupplier, Consumer<AddVisionParams> addVisionMeasurement) {
        super();
        VisionSystem.poseSupplier = poseSupplier;
        this.addVisionMeasurement = addVisionMeasurement;
        simInit = false;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            // should never fail, as WpiLib always provides this file
        }
        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        //stop code from crashing on version mismatch
        PhotonCamera.setVersionCheckEnabled(false);

        cameras = new CameraData[2];

        cameras[0] = new CameraData();
        cameras[0].camera = new PhotonCamera("Shooter_Camera");
        //get the offsets where the camera is mounted
        frontCamPos = new Transform3d(
            new Translation3d(-0.27, -0.3, 0.45), 
            new Rotation3d(Math.PI+Math.toRadians(4),-Math.toRadians(17.5),Math.toRadians(180))
        );
        //get the estimator of it
        cameras[0].poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras[0].camera, frontCamPos);
        cameras[0].poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        cameras[1] = new CameraData();
        cameras[1].camera = new PhotonCamera("Left_Camera");
        //get the offsets where the camera is mounted
        var leftCamPos = new Transform3d(
            new Translation3d(-0.27, 0.3, 0.45), 
            new Rotation3d(Math.PI-Math.toRadians(5),-Math.toRadians(22),Math.toRadians(180))
        );
        //get the estimator of it
        cameras[1].poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras[1].camera, leftCamPos);
        cameras[1].poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        visionStdDev = UtilFunctions.getSettingSub("/Odometry/VisionStdDev", 4);
        lastTargetTimestamp = 0;

        //setup heartbeat
        frontCam = cameras[0].camera;
        heartbeatSub = frontCam.getCameraTable().getIntegerTopic("heartbeat").subscribe(0);
        heartbeatLast = 0;
        heartbeatMisses = 0;
    }

    @Override
    public void periodic() {
        //check if camera is there
        long heartbeat = heartbeatSub.get();
        if(heartbeat != heartbeatLast) {
            heartbeatMisses = 0;
        } else {
            heartbeatMisses++;
        }
        heartbeatLast = heartbeat;

        for(var camera : cameras) {
            processCamera(camera);
        }
        SmartDashboard.putNumber("Robot to Speaker", VisionSystem.getDistanceToTarget(FieldLocation.Speaker));
    }

    private class CameraData {
        PhotonCamera camera;
        PhotonPoseEstimator poseEstimator;
        double lastTimestamp;
    }

    public void processCamera(CameraData data) {
        String name = data.camera.getName();
        // Update and log inputs
        PhotonPipelineResult pipelineResult = data.camera.getLatestResult();
        //need to update our estimate every loop
        data.poseEstimator.setReferencePose(poseSupplier.get());

        //return if we don't have a new packet
        if(data.lastTimestamp == pipelineResult.getLatencyMillis()) return;

        //remove targets too inaccurate to be used
        //have to use a list as .remove() will mess up the for loop as i will increment and skip over records
        LinkedList<PhotonTrackedTarget> toRemove = new LinkedList<PhotonTrackedTarget>();
        for(int i=0; i<pipelineResult.targets.size(); i++) {
            var result = pipelineResult.targets.get(i);
            if (result.getPoseAmbiguity() > APRILTAG_POSE_AMBIGUITY_THRESHOLD) {
                toRemove.add(result);
            }
        }
        pipelineResult.targets.removeAll(toRemove);

        // Return if result is non-existent or invalid
        if (!pipelineResult.hasTargets()) return;

        var target = pipelineResult.getBestTarget();
        SmartDashboard.putNumber(name + " Tag Number", target.getFiducialId());
        SmartDashboard.putNumber(name + " Tag Distance", UtilFunctions.getDistance(target.getBestCameraToTarget()));

        // Update pose estimate
        data.poseEstimator.update(pipelineResult).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;
            // Make sure the measurement is on the field
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= Odometry.FIELD_LENGTH_METERS
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= Odometry.FIELD_WIDTH_METERS) {
                SmartDashboard.putNumber(name + " Pose Data X", estimatedPose.getX());
                SmartDashboard.putNumber(name + " Pose Data Y", estimatedPose.getY());
                SmartDashboard.putNumber(name + " Pose Data Z", estimatedPose.getZ());
                double stdDiv = visionStdDev.get();
                Vector<N3> deviations = VecBuilder.fill(stdDiv, stdDiv, stdDiv);
                var args = new AddVisionParams(estimatedPose.toPose2d(), pipelineResult.getTimestampSeconds(),deviations);
                addVisionMeasurement.accept(args);
                data.lastTimestamp = pipelineResult.getTimestampSeconds();
                lastTargetTimestamp = data.lastTimestamp;
                
                var tag = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if(tag.isPresent()) {
                    SmartDashboard.putNumber(name + " Robot Target Distance", UtilFunctions.getDistance(estimatedPose.toPose2d(), tag.get().toPose2d()));
                }
            }
        });
    }

    @Override
    public void simulationPeriodic() {
        if(simInit == false) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(aprilTagFieldLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(111));
            cameraProp.setCalibError(0.37, 0.13);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            PhotonCameraSim cameraSim = new PhotonCameraSim(frontCam, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, frontCamPos);
            //if you want to see a virtual camera, set this to true, and go to http://localhost:1182/
            cameraSim.enableDrawWireframe(true);
            simInit = true;
        }
        visionSim.update(poseSupplier.get());
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDeviations(Pose2d estimatedPose, PhotonPoseEstimator photonEstimator, PhotonPipelineResult result) {
        Matrix<N3, N1> estStdDeviations;

        var targets = result.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }

        if (numTags == 0) {
            //no tags
            estStdDeviations = kSingleTagStdDeviations;
        } else if (numTags == 1 && avgDist > kMaxVisionDistance) {
            //one tag, but too far.  Making these large so they will be ignored
            estStdDeviations = VecBuilder.fill(1e100, 1e100, 1e100);
        } else if (numTags == 1) {
            //one tag close
            estStdDeviations = kSingleTagStdDeviations;
        } else {
            //multiple tags seen
            avgDist /= numTags;
            estStdDeviations = kMultiTagStdDeviations;
        }

        return estStdDeviations.times(1 + (avgDist * avgDist / 30));
    }

    public boolean isCameraPresent() {
        return heartbeatMisses < 100;
    }

    public boolean seesTarget() {
        //returns if the last time we have seen a target is <1 sec
        return (Timer.getFPGATimestamp() - lastTargetTimestamp) < 1;
    }

    public static Supplier<Pose2d> getLocation(FieldLocation location) {
        switch (location) {
            case Amp:
                return () -> Odometry.mirrorAlliance(new Pose2d(1.8415, 8.2042, Rotation2d.fromDegrees(270)));
            case Speaker:
            default:
                return () -> Odometry.mirrorAlliance(new Pose2d(-0.038099999999999995, 5.547867999999999, Rotation2d.fromDegrees(0)));
        }
    }

    public static double getDistanceToTarget(FieldLocation location) {
        Pose2d pose = poseSupplier.get();
        Pose2d target = VisionSystem.getLocation(location).get();
        return UtilFunctions.getDistance(pose, target);
    }
}
