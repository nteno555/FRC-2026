package frc2025;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.hood.HoodSubsystem;
import frc2025.subsystems.shooter.ShooterSubsystem;
import frc2025.subsystems.vision.LimelightHelpers;
import frc2025.subsystems.vision.VisionPoseEstimator;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class RobotState {
    private static RobotState instance;

    private DriverStation.Alliance alliance;

    private VisionPoseEstimator visionPoseEstimator;
    private double speed;
    private List<VisionPoseEstimator.TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    private ChassisSpeeds robotRelativeSpeeds;
    private ChassisSpeeds fieldRelativeSpeeds;
    private double pigeonAcceleration;
    private double angleToVirtualTarget;

    private boolean manualShuttleOverride;
    private boolean manualShuttle;
    private Pose2d shootingWhileMovingVirtualPose;

    private double turretAngle;
    private double prevTurretAngle;
    private double prevTurretTimestamp;
    private double turretVelocity;

    private double turretMt1AngleRadians;
    private boolean turretMt1Valid;

    public RobotState() {
        visionPoseEstimator = new VisionPoseEstimator(new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 1, 1, 0.00001 }));
        prevTurretTimestamp = -1;
        turretMt1Valid = false;
        turretMt1AngleRadians = 0;
        manualShuttleOverride = false;
    }

    public static RobotState getInstance() {
        if(instance == null) instance = new RobotState();
        return instance;
    }

    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
        robotRelativeSpeeds = speeds;
        fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getVisionPose().getRotation());
    }

    public void setPigeonAcceleration(double accel) { pigeonAcceleration = accel; }
    public double getPigeonAcceleration() { return pigeonAcceleration; }

    public ChassisSpeeds getFieldRelativeSpeeds() {return fieldRelativeSpeeds;}
    public ChassisSpeeds getRobotRelativeSpeeds() {return robotRelativeSpeeds;}

    public Pose2d getShootingWhileMovingVirtualPose() {return shootingWhileMovingVirtualPose;}

    public Pose2d getVisionPose() {
        return visionPoseEstimator.getLatestPose();
    }

    public Pose2d getVisionShooterPose() {
        Pose2d visionPose = visionPoseEstimator.getLatestPose();
        return visionPose.transformBy(SubsystemConstants.robotToShooter);
    }

    public double getSpeed() {
        return Math.hypot(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
    }

    public void rezero(Pose2d pose) {
        visionPoseEstimator.resetPose(pose);
    }

    public void addDriveData(double timestamp, Twist2d twist) {
        visionPoseEstimator.addDriveData(timestamp, twist);
    }

    public void addVisionData(List<VisionPoseEstimator.TimestampedVisionUpdate> update) {
        visionUpdates.addAll(update);
    }

    public void updateVisionPose() {
        visionPoseEstimator.addVisionData(visionUpdates);
        visionUpdates.clear();
    }

    public boolean getShuttling() {
        if (manualShuttleOverride) {
            return manualShuttle;
        }

        if (DriverStation.isAutonomousEnabled()) {
            return false;
        }

        double xPos = RobotState.getInstance().getVisionPose().getX();
        boolean inBlueZone = xPos < FieldConstants.inBlueZoneX && !Alliance.get();
        boolean inRedZone = xPos > FieldConstants.inRedZoneX && Alliance.get();

        Logger.recordOutput("Drive/inBlueZone", inBlueZone);
        Logger.recordOutput("Drive/inRedZone", inRedZone);

        return !inBlueZone && !inRedZone;
    }

    public void setManualShuttleOverride(boolean override) {
        manualShuttleOverride = override;
    }

    public boolean getManualShuttleOverride() {return manualShuttleOverride;}

    public void setManualShuttle(boolean manual) {
        manualShuttle = manual;
    }

    public boolean getManualShuttle() {return manualShuttle;}

    public Pose2d getTargetPose() {
        if (!getShuttling()) {
            return (Alliance.get()) ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
        }

        if (visionPoseEstimator.getLatestPose().getY() > FieldConstants.yHalfLine) {
            return (Alliance.get()) ? FieldConstants.redShuttlePosUpper : FieldConstants.blueShuttlePosUpper;
        }

        return (Alliance.get()) ? FieldConstants.redShuttlePosLower : FieldConstants.blueShuttlePosLower;
    }

    public void updateShootingWhileMovingVirtualPose() {
        Pose2d currentPose = RobotState.getInstance().getVisionPose();
        Pose2d realTargetPose = getTargetPose();
        Pose2d virtualPose = getTargetPose();
        double phaseDelay = 0.0;//0.03;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                currentPose.getRotation()
        );
        double vx = chassisSpeeds.vxMetersPerSecond;
        double vy = chassisSpeeds.vyMetersPerSecond;
        double vOmega = chassisSpeeds.omegaRadiansPerSecond;
        Pose2d delayedPose = currentPose.exp(
                new Twist2d(vx * phaseDelay, vy * phaseDelay, vOmega * phaseDelay)
        );

//        if (SubsystemConstants.hasTurret) {
        Translation2d shooterFieldRelative = SubsystemConstants.robotToShooter.getTranslation().rotateBy(delayedPose.getRotation());
        vx += vOmega * shooterFieldRelative.getY();
        vy += -vOmega * shooterFieldRelative.getX();
//        }

        for (int i = 0; i < 5; i++) {
            double dx = virtualPose.getX() - delayedPose.getX();
            double dy = virtualPose.getY() - delayedPose.getY();
            double distance = Math.hypot(dx, dy);

//            double flightTime = ShooterTreeMap.getInstance().tofTreeMap.getInterpolated(distance);
            double flightTime = ShooterTreeMap.getInstance().getRealTof(distance);

            virtualPose = new Pose2d(
                    realTargetPose.getX() - vx * flightTime,
                    realTargetPose.getY() - vy * flightTime,
                    realTargetPose.getRotation()
            );
        }
        shootingWhileMovingVirtualPose = virtualPose;

        Pose2d shooterPose = delayedPose.transformBy(SubsystemConstants.robotToShooter);
        angleToVirtualTarget = Math.atan2(
                virtualPose.getY() - shooterPose.getY(),
                virtualPose.getX() - shooterPose.getX()
        );
        Logger.recordOutput("Drive/speed", Math.hypot(vx, vy));
        Logger.recordOutput("Vision/shootingWhileMovingVirtualPose", shootingWhileMovingVirtualPose);
        Logger.recordOutput("Vision/angleToVirtualTarget", angleToVirtualTarget);
    }

    public double getAngleToVirtualTarget() {
        return angleToVirtualTarget;
    }

    public void setTurretAngle(double angleRadians) {
        prevTurretAngle = turretAngle;
        prevTurretTimestamp = Utils.getCurrentTimeSeconds();
        turretAngle = angleRadians;
    }

    public void setTurretVelocity(double vel) {
        turretVelocity = vel;
    }

    public double getTurretVelocity() {
       if (prevTurretTimestamp == -1) return 0.0;
       return (turretAngle - prevTurretAngle)/(Utils.getCurrentTimeSeconds() - prevTurretTimestamp);
//        return turretVelocity;
    }

    public double getTurretAngle() { return turretAngle; }

    public double getRadialVelocity() {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                getVisionPose().getRotation()
        );

        Translation2d velocity = new Translation2d(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond
        );
        Translation2d target = getTargetPose().getTranslation().minus(getVisionShooterPose().getTranslation())
                .div(getTargetPose().getTranslation().minus(getVisionShooterPose().getTranslation()).getNorm());

        return velocity.getX()*target.getX() + velocity.getY()*target.getY();

    }

    public void setTurretMt1AngleRadians(double angle) { turretMt1AngleRadians = angle; }
    public double getTurretMt1AngleRadians() { return turretMt1AngleRadians; }
    public void setTurretMt1Valid(boolean valid) { turretMt1Valid = valid; }
    public boolean getTurretMt1Valid() { return turretMt1Valid; }
}
