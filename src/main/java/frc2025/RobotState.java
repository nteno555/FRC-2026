package frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc2025.config.FieldConstants;
import frc2025.subsystems.vision.VisionPoseEstimator;
import frc2025.util.Alliance;

import java.util.ArrayList;
import java.util.List;

public class RobotState {
    private static RobotState instance;

    private Pose2d visionPose;
    private double speed;
    private List<VisionPoseEstimator.TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

    private boolean shuttling = false;

    public static RobotState getInstance() {
        if(instance == null) instance = new RobotState();
        return instance;
    }

    public void setVisionPose(Pose2d visionPose) {
        this.visionPose = visionPose;
    }
    public Pose2d getVisionPose() {
        return visionPose;
    }

    public void setSpeed(double speed) { this.speed = speed; }
    public double getSpeed() { return this.speed; }

    public void addVisionData(List<VisionPoseEstimator.TimestampedVisionUpdate> update) {
        visionUpdates.addAll(update);
    }
    public List<VisionPoseEstimator.TimestampedVisionUpdate> getVisionUpdates() {
        return this.visionUpdates;
    }
    public void clearVisionUpdates() {
        this.visionUpdates.clear();
    }

    public void setShuttling(boolean shuttling) {
        this.shuttling = shuttling;
    }
    public boolean getShuttling() {
        return this.shuttling;
    }
    public Pose2d getTargetPose() {
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            if(shuttling) {
                return FieldConstants.redNetPos;
            } else {
                if(visionPose.getY() > FieldConstants.yHalfLine) {
                    return FieldConstants.redShuttlePosUpper;
                } else {
                    return FieldConstants.redShuttlePosLower;
                }
            }
        } else {
            if(shuttling) {
                return FieldConstants.blueNetPos;
            } else {
                if(visionPose.getY() > FieldConstants.yHalfLine) {
                    return FieldConstants.blueShuttlePosUpper;
                } else {
                    return FieldConstants.blueShuttlePosLower;
                }
            }
        }
    }
}
