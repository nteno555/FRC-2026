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
// import frc2025.subsystems.hood.HoodSubsystem;
// import frc2025.subsystems.shooter.ShooterSubsystem;
import frc2025.subsystems.vision.LimelightHelpers;
import frc2025.subsystems.vision.VisionPoseEstimator;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class RobotState {
    //private static double velocitySampleDt = 0.05;

    private static RobotState instance;

    private Pose2d visionPoseBuiltin;
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
    private boolean manual;
    private Pose2d shootingWhileMovingVirtualPose;

    private boolean driveShotReady;
    private boolean shooterReady;
    private boolean hoodReady;
    private boolean driveAutoAiming;
    private boolean wantAutoIntaking;

    private Pose2d turretLimelightPose;
    private Translation3d shotVectorWhileMoving = new Translation3d(0, 0, 0);

    private Pose2d currentVisionPose;
    private double prevVisionPoseTimestamp;
    private Pose2d prevVisionPose;

    private List<PhotonTrackedTarget> ballTargets;
//    private double targetRadialVelocity;
//    private double targetDistance;

//    private double shooterExitVelocity;
//    private double hoodAngle;

    public RobotState() {
        visionPoseEstimator = new VisionPoseEstimator(new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 1, 1, 0.00001 }));
        shuttling = false;
        manual = false;
        prevFieldRelativeSpeedTimestamp = Utils.getCurrentTimeSeconds();
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
    public ChassisSpeeds getPrevFieldRelativeSpeeds() {return prevFieldRelativeSpeeds;}
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
        Translation2d vec = SubsystemConstants.shooterToCenterPose.getTranslation().rotateBy(visionPose.getRotation());
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
//                    return new Pose2d(
//                            FieldConstants.redShuttlePosUpper.getX(),
//                            8 - visionPoseEstimator.getLatestPose().getY() + 4 - 0.3,
//                            Rotation2d.kZero
//                    );
                } else {
                    return FieldConstants.redShuttlePosLower;
//                    return new Pose2d(
//                            FieldConstants.redShuttlePosLower.getX(),
//                            4 - visionPoseEstimator.getLatestPose().getY() + 0.3,
//                            Rotation2d.kZero
//                    );
                }
            }
        } else {
            if(!shuttling) {
                return FieldConstants.blueNetPos;
            } else {
                if(visionPoseEstimator.getLatestPose().getY() > FieldConstants.yHalfLine) {
                    return FieldConstants.blueShuttlePosUpper;
//                    return new Pose2d(
//                            FieldConstants.blueShuttlePosUpper.getX(),
//                            8 - visionPoseEstimator.getLatestPose().getY() + 4 - 0.3,
//                            Rotation2d.kZero
//                    );
                }
                else {
                    return FieldConstants.blueShuttlePosLower;
//                    return new Pose2d(
//                            FieldConstants.blueShuttlePosLower.getX(),
//                            4 - visionPoseEstimator.getLatestPose().getY() + 0.3,
////                            -FieldConstants.yHalfLine + visionPoseEstimator.getLatestPose().getY(),
//                            Rotation2d.kZero
//                    );
                }
            }
        }
    }

//     public void updateShootingWhileMovingVirtualPose() {
// //        currentVisionPose = getVisionPose();
// //        Translation2d velocity;
// //
// //        if(prevVisionPose == null) {
// //            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
// //                robotRelativeSpeeds,
// //                currentVisionPose.getRotation()
// //            );
// //            velocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
// //        } else {
// //            velocity = new Translation2d(
// //                    currentVisionPose.getX() - prevVisionPose.getX(),
// //                    currentVisionPose.getY() - prevVisionPose.getY()
// //            ).div(Utils.getCurrentTimeSeconds() - prevVisionPoseTimestamp);
// //        }
// //
// //        prevVisionPose = currentVisionPose;
// //        prevVisionPoseTimestamp = Utils.getCurrentTimeSeconds();

//         Pose2d currentPose = RobotState.getInstance().getVisionPose();
//         //Pose2d prevPose = sampleVisionPose();
//         Pose2d realTargetPose = getTargetPose();
//         Pose2d virtualPose = getTargetPose();

//         ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
//                 robotRelativeSpeeds,
//                 currentPose.getRotation()
//         );

//         double vx = chassisSpeeds.vxMetersPerSecond;
//         double vy = chassisSpeeds.vyMetersPerSecond;
//         Translation2d velocity = new Translation2d(vx, vy);
// //        Translation2d velocity = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

//         for (int i = 0; i < 10; i++) {
//             double dx = virtualPose.getX() - currentPose.getX();
//             double dy = virtualPose.getY() - currentPose.getY();
//             double distance = Math.hypot(dx, dy);

//             double flightTime = ShooterTreeMap.getInstance().tofTreeMap.getInterpolated(distance);

//             virtualPose = new Pose2d(
//                     realTargetPose.getX() + velocity.getX() * flightTime,
//                     realTargetPose.getY() + velocity.getY() * flightTime,
//                     realTargetPose.getRotation()
//             );
//         }

// //        Vector<N2> velocityRelToTarget = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond).toVector();
// //        velocityRelToTarget = velocityRelToTarget.projection(virtualPose.getTranslation().toVector());
// //

//         shootingWhileMovingVirtualPose = virtualPose;

//         Pose2d shooterPose = getVisionShooterPose();
//         angleToVirtualTarget = Math.atan2(
//                 virtualPose.getY() - shooterPose.getY(),
//                 virtualPose.getX() - shooterPose.getX()
//         );
//         Logger.recordOutput("Drive/speed", velocity.getNorm());
//         Logger.recordOutput("Vision/shootingWhileMovingVirtualPose", shootingWhileMovingVirtualPose);
//         Logger.recordOutput("Vision/angleToVirtualTarget", angleToVirtualTarget);
//     }

    public double getAngleToVirtualTarget() {
        return angleToVirtualTarget;
    }

    public void setDriveShotReady(boolean ready) { driveShotReady = ready; }
    public boolean getDriveShotReady() { return driveShotReady; }
    public void setShooterReady(boolean ready) { shooterReady = ready; }
    public boolean getShooterReady() { return shooterReady; }
    public void setDriveAutoAiming(boolean ready) { driveAutoAiming = ready; }
    public boolean getDriveAutoAiming() { return driveAutoAiming; }
    public void setWantAutoIntaking(boolean want) {wantAutoIntaking = want;}
    public boolean getWantAutoIntaking() {return wantAutoIntaking;}
    public void setManual(boolean enable) {manual = enable;}
    public boolean getManual() {return manual;}
    public void setTurretLimelightPose(Pose2d pose) { turretLimelightPose = pose; }
    public Pose2d getTurretLimelightPose() { return turretLimelightPose; }

    public List<PhotonTrackedTarget> getBallTargets() {
        return this.ballTargets;
    }
    
    public void setBallTarget(List<PhotonTrackedTarget> targets) {
        this.ballTargets = targets;
    }

//    public void setShooterExitVelocity(double velocity) { shooterExitVelocity = velocity; }
//    public double getShooterExitVelocity() { return shooterExitVelocity; }
//    public void setHoodAngle(double angle) { hoodAngle = angle; }
//    public double getHoodAngle() { return hoodAngle; }

    // public Translation3d getShotVector(double shooterExitVelocity, double hoodAngle) { // {field x, field y, height}
    //     Translation2d direction = getTargetPose().getTranslation().minus(getVisionShooterPose().getTranslation());
    //     direction = direction.div(direction.getNorm());

    //     Translation2d xyVelocity = direction.times(shooterExitVelocity * Math.cos(hoodAngle));

    //     double zSpeed = shooterExitVelocity * Math.sin(hoodAngle);

    //     return new Translation3d(
    //             xyVelocity.getX(),
    //             xyVelocity.getY(),
    //             zSpeed
    //     );
    // }

//     public void updateShotVectorWhileMoving() {
//         ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
//                 robotRelativeSpeeds,
//                 getVisionPose().getRotation()
//         );

//         Vector<N2> velocityVector = VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
//         Vector<N2> directionToTarget = VecBuilder.fill(
//                 getTargetPose().getX() - getVisionShooterPose().getX(),
//                 getTargetPose().getY() - getVisionShooterPose().getY()
//         );
//         directionToTarget = directionToTarget.div(directionToTarget.norm());

//         Vector<N2> radialVelocity = velocityVector.projection(directionToTarget);
//         double targetRadialVelocity = velocityVector.dot(directionToTarget);
//         double targetDistance = Math.hypot(getTargetPose().getX() - getVisionShooterPose().getX(),
//                 getTargetPose().getY() - getVisionShooterPose().getY());
//         Vector<N2> tangentialVelocity = velocityVector.minus(radialVelocity);
// //        if (tangentialVelocity.norm() < 0.05) {
//             tangentialVelocity = VecBuilder.fill(0.0, 0.0);
// //        }

// //        double shooterSpeed = ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(distance);
// //        double hoodAngle = ShooterTreeMap.getInstance().hoodTreeMap.getInterpolated(distance);
// //        double shooterSpeed = ShooterTreeMap.getInstance().calculateRps(targetRadialVelocity, targetDistance);
// //        double hoodAngle = ShooterTreeMap.getInstance().calculateHoodAngle(targetRadialVelocity, targetDistance);
//         double shooterSpeed = ShooterTreeMap.getInstance().calculateRps(0.0, targetDistance);
//         double hoodAngle = ShooterTreeMap.getInstance().calculateHoodAngle(0.0, targetDistance);

//         Translation3d shotVector = getShotVector(ShooterSubsystem.toBallExitVelocity(shooterSpeed), HoodSubsystem.toRadians(hoodAngle));


// //        shotVectorWhileMoving = new Translation3d(
// //                shotVector.getX() - chassisSpeeds.vxMetersPerSecond,
// //                shotVector.getY() - chassisSpeeds.vyMetersPerSecond,
// //                shotVector.getZ()
// //        );

//         shotVectorWhileMoving = new Translation3d(
//                 shotVector.getX() - tangentialVelocity.get(0),
//                 shotVector.getY() - tangentialVelocity.get(1),
//                 shotVector.getZ()
//         );

//     }

    // public Translation3d getShotVectorWhileMoving() {
    //     return shotVectorWhileMoving;
    // }
}
