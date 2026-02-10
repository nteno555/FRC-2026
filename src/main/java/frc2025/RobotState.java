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
    private ChassisSpeeds prevFieldRelativeSpeeds;
    private ChassisSpeeds fieldRelativeSpeeds;
    private ChassisSpeeds fieldRelativeAcceleration;
    private double prevFieldRelativeSpeedTimestamp;
    private double angleToVirtualTarget;

    private boolean shuttling;
    private Pose2d shootingWhileMovingVirtualPose;

    private boolean driveShotReady;
    private boolean shooterReady;
    private boolean hoodReady;
    private boolean driveAutoAiming;
    private boolean wantAutoIntaking;
    private boolean wantAutoShoot;

    private double turretAngle;
    private double prevTurretAngle;
    private double prevTurretTimestamp;

    public RobotState() {
        visionPoseEstimator = new VisionPoseEstimator(new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 1, 1, 0.00001 }));
        shuttling = false;
        prevFieldRelativeSpeedTimestamp = Utils.getCurrentTimeSeconds();
        prevTurretTimestamp = -1;
    }

    public static RobotState getInstance() {
        if(instance == null) instance = new RobotState();
        return instance;
    }

    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
        double now = Utils.getCurrentTimeSeconds();

        robotRelativeSpeeds = speeds;
        fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getVisionPose().getRotation());

        if (prevFieldRelativeSpeeds != null) {
            double vx = fieldRelativeSpeeds.vxMetersPerSecond - prevFieldRelativeSpeeds.vxMetersPerSecond;
            double vy = fieldRelativeSpeeds.vyMetersPerSecond - prevFieldRelativeSpeeds.vyMetersPerSecond;
            double theta = fieldRelativeSpeeds.omegaRadiansPerSecond - prevFieldRelativeSpeeds.omegaRadiansPerSecond;
            fieldRelativeAcceleration = new ChassisSpeeds(vx, vy, theta).div(now - prevFieldRelativeSpeedTimestamp);
        }

        prevFieldRelativeSpeedTimestamp = now;
        prevFieldRelativeSpeeds = fieldRelativeSpeeds;

        Logger.recordOutput("Drive/fieldRelativeAccel", fieldRelativeAcceleration);
    }

    public ChassisSpeeds getFieldRelativeAcceleration() {return fieldRelativeAcceleration;}
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
        visionPoseEstimator.addVisionData(update);
    }

    public void setShuttling(boolean shuttling) {
        this.shuttling = shuttling;
    }
    public boolean getShuttling() {
        return this.shuttling;
    }

    public Pose2d getTargetPose() {
        alliance = DriverStation.Alliance.Red;
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
                }
                else {
                    return FieldConstants.blueShuttlePosLower;
                }
            }
        }
    }

    public void updateShuttling() {
        double xPos = RobotState.getInstance().getVisionPose().getX();

        if (DriverStation.isAutonomousEnabled()) {
            shuttling = false;
            return;
        }

        if (xPos > 4.6 && xPos < 11.9) shuttling = true;
        else shuttling = false;
    }

    public void updateShootingWhileMovingVirtualPose() {
        Pose2d currentPose = RobotState.getInstance().getVisionPose();
        Pose2d realTargetPose = getTargetPose();
        Pose2d virtualPose = getTargetPose();
        double phaseDelay = 0.03;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                currentPose.getRotation()
        );
        double vx = chassisSpeeds.vxMetersPerSecond;
        double vy = chassisSpeeds.vyMetersPerSecond;
        double vOmega = chassisSpeeds.omegaRadiansPerSecond;
        Translation2d velocity = new Translation2d(vx, vy);
        Pose2d delayedPose = currentPose.exp(
                new Twist2d(vx * phaseDelay, vy * phaseDelay, vOmega * phaseDelay)
        );

        for (int i = 0; i < 20; i++) {
            double dx = virtualPose.getX() - delayedPose.getX();
            double dy = virtualPose.getY() - delayedPose.getY();
            double distance = Math.hypot(dx, dy);

            double flightTime = ShooterTreeMap.getInstance().tofTreeMap.getInterpolated(distance);

            virtualPose = new Pose2d(
                    realTargetPose.getX() + velocity.getX() * flightTime,
                    realTargetPose.getY() + velocity.getY() * flightTime,
                    realTargetPose.getRotation()
            );
        }
        shootingWhileMovingVirtualPose = virtualPose;

        Pose2d shooterPose = delayedPose.transformBy(SubsystemConstants.robotToShooter);
        angleToVirtualTarget = Math.atan2(
                virtualPose.getY() - shooterPose.getY(),
                virtualPose.getX() - shooterPose.getX()
        );
        Logger.recordOutput("Drive/speed", velocity.getNorm());
        Logger.recordOutput("Vision/shootingWhileMovingVirtualPose", shootingWhileMovingVirtualPose);
        Logger.recordOutput("Vision/angleToVirtualTarget", angleToVirtualTarget);
    }

    public double getAngleToVirtualTarget() {
        return angleToVirtualTarget;
    }

    public boolean getWantAutoIntaking() {return wantAutoIntaking;}
    public void setWantAutoShoot(boolean want) {wantAutoShoot = want;}
    public boolean getWantAutoShoot() {return wantAutoShoot;}

    public void setTurretAngle(double angleRadians) {
        prevTurretAngle = turretAngle;
        prevTurretTimestamp = Utils.getCurrentTimeSeconds();
        turretAngle = angleRadians;
    }

    public double getTurretVelocity() {
       if (prevTurretTimestamp == -1) return 0.0;
       return (turretAngle - prevTurretAngle)/(Utils.getCurrentTimeSeconds() - prevTurretTimestamp);
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

}
