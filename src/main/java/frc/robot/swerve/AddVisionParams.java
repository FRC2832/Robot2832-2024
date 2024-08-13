package frc.robot.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class AddVisionParams {
    public Pose2d pose;
    public double timestamp;
    public Matrix<N3, N1> stdDeviations;

    public AddVisionParams(Pose2d pose, double timestamp, Matrix<N3, N1> stdDeviations) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.stdDeviations = stdDeviations;
    }
}
