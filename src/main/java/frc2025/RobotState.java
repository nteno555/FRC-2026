package frc2025;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.vision.VisionPoseEstimator;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class RobotState {
    private static RobotState instance;

    private Pose2d visionPoseBuiltin;
    private VisionPoseEstimator visionPoseEstimator;
    private double speed;
    private List<VisionPoseEstimator.TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    private ChassisSpeeds robotRelativeSpeeds;
    private double angleToVirtualTarget;
    private Translation2d robotAcceleration = new Translation2d();

    private boolean shuttling;
    private Pose2d shootingWhileMovingVirtualPose;

    private boolean driveShotReady;
    private boolean shooterReady;
    private boolean hoodReady;

    public RobotState() {
        visionPoseEstimator = new VisionPoseEstimator(new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 1, 1, 0.00001 }));
        shuttling = false;
    }

    public static RobotState getInstance() {
        if(instance == null) instance = new RobotState();
        return instance;
    }

    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) { robotRelativeSpeeds = speeds; }
    public ChassisSpeeds getRobotRelativeSpeeds() {return robotRelativeSpeeds;}

    public Pose2d getShootingWhileMovingVirtualPose() {return shootingWhileMovingVirtualPose;}

    public void setVisionPoseBuiltin(Pose2d visionPoseBuiltin) {
        this.visionPoseBuiltin = visionPoseBuiltin;
    }
    public Pose2d getVisionPoseBuiltin() { return this.visionPoseBuiltin; }

    public Pose2d getVisionPose() {
        return visionPoseEstimator.getLatestPose();
    }

    public Pose2d getVisionShooterPose() {
        Pose2d visionPose = visionPoseEstimator.getLatestPose();
        Translation2d vec = SubsystemConstants.shooterToCenterPose.rotateBy(visionPose.getRotation());
        return new Pose2d(
                visionPose.getX() + vec.getX(),
                visionPose.getY() + vec.getY(),
                visionPose.getRotation()
        );
    }

    public void setSpeed(double speed) { this.speed = speed; }
    public double getSpeed() { return this.speed; }

    public void rezero(Pose2d pose) {
        visionPoseEstimator.resetPose(pose);
    }

    public void addDriveData(double timestamp, Twist2d twist) {
        visionPoseEstimator.addDriveData(timestamp, twist);
    }

    public void addVisionData(List<VisionPoseEstimator.TimestampedVisionUpdate> update) {
        visionUpdates.addAll(update);
        visionPoseEstimator.addVisionData(update);
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
        var alliance = DriverStation.Alliance.Red;
        if(DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

        if(alliance == DriverStation.Alliance.Red) {
            if(!shuttling) {
                return FieldConstants.redNetPos;
            } else {
                if(visionPoseEstimator.getLatestPose().getY() > FieldConstants.yHalfLine) {
                    return FieldConstants.redShuttlePosUpper;
                } else {
                    return FieldConstants.redShuttlePosLower;
                }
            }
        } else {
            if(!shuttling) {
                return FieldConstants.blueNetPos;
            } else {
                if(visionPoseEstimator.getLatestPose().getY() > FieldConstants.yHalfLine) {
                    return FieldConstants.blueShuttlePosUpper;
                } else {
                    return FieldConstants.blueShuttlePosLower;
                }
            }
        }
    }

    public void updateShootingWhileMovingVirtualPose() {
        Pose2d currentPose = RobotState.getInstance().getVisionPose();
        Pose2d realTargetPose = getTargetPose();
        Pose2d virtualPose = getTargetPose();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                currentPose.getRotation()
        );
        Translation2d velocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

        for (int i = 0; i < 5; i++) {
            double dx = virtualPose.getX() - currentPose.getX();
            double dy = virtualPose.getY() - currentPose.getY();
            double distance = Math.hypot(dx, dy);

            double flightTime = ShooterTreeMap.getInstance().tofTreeMap.getInterpolated(distance);

            virtualPose = new Pose2d(
                    realTargetPose.getX() + velocity.getX() * flightTime + 0.5 * robotAcceleration.getX() * Math.pow(flightTime,2),
                    realTargetPose.getY() + velocity.getY() * flightTime + 0.5 * robotAcceleration.getY() * Math.pow(flightTime,2),
                    realTargetPose.getRotation()
            );
        }

        shootingWhileMovingVirtualPose = virtualPose;

        Pose2d shooterPose = getVisionShooterPose();
        angleToVirtualTarget = Math.atan2(
                virtualPose.getY() - shooterPose.getY(),
                virtualPose.getX() - shooterPose.getX()
        );
        Logger.recordOutput("Vision/shootingWhileMovingVirtualPose", shootingWhileMovingVirtualPose);
    }

    public void setRobotAcceleration(Translation2d robotAcceleration) {
        this.robotAcceleration = robotAcceleration;
    }

    public double getAngleToVirtualTarget() {
        return angleToVirtualTarget;
    }

    public void setDriveShotReady(boolean ready) { driveShotReady = ready; }
    public boolean getDriveShotReady() { return driveShotReady; }
    public void setShooterReady(boolean ready) { shooterReady = ready; }
    public boolean getShooterReady() { return shooterReady; }
}
