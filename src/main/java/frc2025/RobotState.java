package frc2025;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class RobotState {
    private static RobotState instance;

    private DriverStation.Alliance alliance;

    public static record TimestampedVisionUpdate(
            double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {
    }

    private Pose2d latestPose = new Pose2d();

    private List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();

    private ChassisSpeeds robotRelativeSpeeds;

    private ChassisSpeeds fieldRelativeSpeeds;

    private double pigeonAcceleration;

    public RobotState() {
        latestPose = new Pose2d();
    }

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
        robotRelativeSpeeds = speeds;
        fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getVisionPose().getRotation());
    }

    public void setPigeonAcceleration(double accel) {
        pigeonAcceleration = accel;
    }

    public double getPigeonAcceleration() {
        return pigeonAcceleration;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return fieldRelativeSpeeds;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return robotRelativeSpeeds;
    }

    public Pose2d getVisionPose() {
        return latestPose;
    }

    public void setPose(Pose2d pose) {
        latestPose = pose;
    }

    public Pose2d getVisionShooterPose() {
        Pose2d visionPose = getVisionPose();
        return visionPose.transformBy(SubsystemConstants.robotToShooter);
    }

    public double getSpeed() {
        return Math.hypot(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    }

    public void rezero(Pose2d pose) {
        latestPose = pose;
        visionUpdates.clear();
    }

    public void addDriveData(double timestamp, Twist2d twist) {
    }

    public void addVisionData(List<TimestampedVisionUpdate> update) {
        visionUpdates.addAll(update);
    }

    public List<TimestampedVisionUpdate> getVisionUpdates() {
        var updates = new ArrayList<>(visionUpdates);
        visionUpdates.clear();
        return updates;
    }

    public void updateVisionPose() {
        visionUpdates.clear();
    }


 
}