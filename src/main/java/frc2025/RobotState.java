package frc2025;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N2;
import frc2025.config.FieldConstants;
import frc2025.subsystems.vision.VisionPoseEstimator;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class RobotState {
    private VisionPoseEstimator visionPoseEstimator;
    private Pose2d visionPose;
    private Pose2d swervePose;
    private static RobotState instance;
    private double alignDist;
    private double speed;

    private double modAngle(double angle) {
        if(angle > Math.PI) return angle - 2*Math.PI;
        return angle;
    }


    public RobotState() {
        visionPoseEstimator = new VisionPoseEstimator(new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 1, 1, 0.00001 }));
    }

    public static RobotState getInstance() {
        if(instance == null) instance = new RobotState();
        return instance;
    }

    public void addDriveData(double timestamp, Twist2d twist) {
        visionPoseEstimator.addDriveData(timestamp, twist);
        visionPose = visionPoseEstimator.getLatestPose();
    }

    public void addVisionData(List<VisionPoseEstimator.TimestampedVisionUpdate> update) {
        visionPoseEstimator.addVisionData(update);
        visionPose = visionPoseEstimator.getLatestPose();
    }

    public void rezero(Pose2d pose) {
        swervePose = pose;
        visionPose = pose;
        visionPoseEstimator.resetPose(pose);
    }

    public void setSwervePose(Pose2d pose) {
        swervePose = pose;
    }

    public Pose2d getVisionPose() { return visionPoseEstimator.getLatestPose(); }
    public Pose2d getSwervePose() { return swervePose; }

    public double getAngleToTarget() {
        Pose2d robotPose = RobotState.getInstance().getVisionPose();
        Pose2d targetPose = Alliance.get() ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
        double angleToTarget = Math.atan2(
                targetPose.getY() - robotPose.getY(),
                targetPose.getX() - robotPose.getX()
        ) + robotPose.getRotation().getRadians();
        return modAngle(angleToTarget);
    }

    public void setAlignDist(double alignDist) {
        this.alignDist = alignDist;
    }
    public double getAlignDist() { return this.alignDist; }
    public void setSpeed(double speed) { this.speed = speed; }
    public double getSpeed() { return this.speed; }
}
