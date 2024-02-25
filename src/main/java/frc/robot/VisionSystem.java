package frc.robot;

import java.io.IOException;
import java.util.LinkedList;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSystem extends SubsystemBase {
    Odometry odometry;
    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator frontCamEstimator;
    private VisionSystemSim visionSim;
    private boolean simInit;

    private PhotonCamera frontCam;
    private Transform3d frontCamPos;
    private double lastTimestamp;
    private final double APRILTAG_POSE_AMBIGUITY_THRESHOLD = 0.2;
    
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDeviations = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDeviations = VecBuilder.fill(0.5, 0.5, 1);
    public static final double kMaxVisionDistance = 4;
    private DoubleSubscriber visionStdDev;

    public VisionSystem(Odometry odometry) {
        super();
        this.odometry = odometry;
        simInit = false;
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            // should never fail, as WpiLib always provides this file
        }
        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        //get camera by name
        frontCam = new PhotonCamera("Shooter_Camera");
        //get the offsets where the camera is mounted
        frontCamPos = new Transform3d(
            new Translation3d(-0.31, -0.29, 0.47), 
            new Rotation3d(0,Math.toRadians(11.5),Math.toRadians(180))
        );
        //get the estimator of it
        frontCamEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam, frontCamPos);
        frontCamEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        visionStdDev = UtilFunctions.getSettingSub("/Odometry/VisionStdDev", 4);
        lastTimestamp = 0;
    }

    @Override
    public void periodic() {
        // Update and log inputs
        PhotonPipelineResult pipelineResult = frontCam.getLatestResult();
        //need to update our estimate every loop
        frontCamEstimator.setReferencePose(odometry.getPose());

        //return if we don't have a new packet
        if(lastTimestamp == pipelineResult.getLatencyMillis()) return;

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

        // Update pose estimate
        frontCamEstimator.update(pipelineResult).ifPresent(estimatedRobotPose -> {
            var estimatedPose = estimatedRobotPose.estimatedPose;
            // Make sure the measurement is on the field
            if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= Odometry.FIELD_LENGTH_METERS
                && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= Odometry.FIELD_WIDTH_METERS) {
                SmartDashboard.putNumber("Pose Data X", estimatedPose.getX());
                SmartDashboard.putNumber("Pose Data Y", estimatedPose.getY());
                SmartDashboard.putNumber("Pose Data Z", estimatedPose.getZ());
                double stdDiv = visionStdDev.get();
                Vector<N3> deviations = VecBuilder.fill(stdDiv, stdDiv, stdDiv);
                odometry.addVisionMeasurement(estimatedPose.toPose2d(), pipelineResult.getTimestampSeconds(),deviations);
                lastTimestamp = pipelineResult.getTimestampSeconds();
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
        visionSim.update(odometry.getPose());
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
}
