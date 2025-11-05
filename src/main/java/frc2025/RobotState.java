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

    public double getReefDepth() {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        double minDepth = 999999;
        double minDist = 99999;
        for(var tag: layout.getTags()) {
            Pose2d tagPose = tag.pose.toPose2d();
            Translation2d robotToTag = visionPose.getTranslation().minus(tagPose.getTranslation());
            Vector<N2> translation = VecBuilder.fill(robotToTag.getX(), robotToTag.getY());
            Vector<N2> heading = VecBuilder.fill(Math.cos(tagPose.getRotation().getRadians()), Math.sin(tagPose.getRotation().getRadians()));
            double dist = robotToTag.getNorm();
            if(dist < minDist) {
                minDist = dist;
                minDepth = translation.projection(heading).norm();
            }
        }
        Logger.recordOutput("Vision/reefDepth", minDepth);
        return minDepth;
    }

    public Translation2d getReefDepthVector() {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        Translation2d minVector = new Translation2d();
        double minDepth = 999999;
        for(var tag: layout.getTags()) {
            Pose2d tagPose = tag.pose.toPose2d();
            Translation2d robotToTag = visionPose.getTranslation().minus(tagPose.getTranslation());
            Vector<N2> translation = VecBuilder.fill(robotToTag.getX(), robotToTag.getY());
            Vector<N2> heading = VecBuilder.fill(Math.cos(visionPose.getRotation().getRadians()), Math.sin(visionPose.getRotation().getRadians()));

            Vector<N2> vec = translation.projection(heading);
            if(vec.norm() < minDepth) {
                minDepth = vec.norm();
                minVector = new Translation2d(vec.get(0), vec.get(1));
            }
        }
        Logger.recordOutput("Vision/reefDepth", minDepth);
        return minVector;
    }

    public double getNetDepth() {
        double netX = FieldConstants.blueNetPos.getX();
        if(Alliance.get()) netX = FieldConstants.redNetPos.getX();

        double res = netX - visionPose.getX();
        if(Alliance.get()) res = res * -1;
        return res;
    }

    public void setAlignDist(double alignDist) {
        this.alignDist = alignDist;
    }

    public double getAlignDist() { return this.alignDist; }

    public void setSpeed(double speed) { this.speed = speed; }
    public double getSpeed() { return this.speed; }
}
