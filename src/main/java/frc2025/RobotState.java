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
    private static RobotState instance;

    private Pose2d visionPose;
    private Pose2d swervePose;
    private double alignDist;
    private double speed;
    private List<VisionPoseEstimator.TimestampedVisionUpdate> lastVisionUpdate;
    private boolean hasNewVisionUpdate = false;

    private double modAngle(double angle) {
        if(angle > Math.PI) return angle - 2*Math.PI;
        return angle;
    }

    public static RobotState getInstance() {
        if(instance == null) instance = new RobotState();
        return instance;
    }



    public void rezero(Pose2d pose) {
        swervePose = pose;
        //visionPoseEstimator.resetPose(pose);
    }

    public void setSwervePose(Pose2d pose) {
        swervePose = pose;
    }
    public void setVisionPose(Pose2d visionPose) {
        this.visionPose = visionPose;
    }
    public Pose2d getVisionPose() {
        return visionPose;
    }
    public Pose2d getSwervePose() {
        return swervePose;
    }

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

    public void addVisionData(List<VisionPoseEstimator.TimestampedVisionUpdate> update) {
        lastVisionUpdate = update;
        this.hasNewVisionUpdate = true;
    }
    public List<VisionPoseEstimator.TimestampedVisionUpdate> getLastVisionUpdate() {
        this.hasNewVisionUpdate = false;
        return this.lastVisionUpdate;
    }
    //public void setHasNewVisionUpdate(boolean hasNew) {
     //   this.hasNewVisionUpdate = hasNew;
    //}
    public boolean getHasNewVisionUpdate() {
        return this.hasNewVisionUpdate;
    }
}
