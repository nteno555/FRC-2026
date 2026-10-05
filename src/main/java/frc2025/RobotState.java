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
import org.photonvision.targeting.PhotonTrackedTarget;

import java.sql.Driver;
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

    private Pose2d shootingWhileMovingVirtualPose;

    private List<PhotonTrackedTarget> ballTargets;

    private double angleToVirtualTarget;

    private double prevAngleToVirtualTarget = 0.0;
    private double prevVirtualTargetTimestamp = -1.0;
    private double virtualTargetAngularVelocity = 0.0;
    private LinearFilter virtualTargetVelocityFilter = LinearFilter.singlePoleIIR(0.03, 0.02);

    private double prevWantedShooterRPS = 0.0;
    private double wantedShooterAcceleration = 0.0;
    private LinearFilter shooterAccelerationFilter = LinearFilter.singlePoleIIR(0.03, 0.02);

    private double prevWantedHoodRotation = 0.0;
    private double wantedHoodVelocity = 0.0;
    private LinearFilter hoodVelocityFilter = LinearFilter.singlePoleIIR(0.03, 0.02);

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
    public Pose2d getDelayedShooterPose(){
        return getDelayedPose().transformBy(SubsystemConstants.robotToShooter);
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

    public void setBallTargets(List<PhotonTrackedTarget> targets) {
        this.ballTargets = targets;
    }

    public List<PhotonTrackedTarget> getBallTargets() {
        return this.ballTargets;
    }

    public double getDistanceToHub() {
        double distance = 0.0;
        if (Alliance.get()) {
            distance = Math.sqrt(Math.pow((FieldConstants.redNetPos.getY() - getVisionPose().getY()), 2) + Math.pow((FieldConstants.redNetPos.getX() - getVisionPose().getX()), 2));  
        } else {
            distance = Math.sqrt(Math.pow((FieldConstants.blueNetPos.getY() - getVisionPose().getY()), 2) + Math.pow((FieldConstants.blueNetPos.getX() - getVisionPose().getX()), 2));  
        }
        return distance;
    }
    public ArrayList<Pose2d> getBallFieldPoses() {
        ArrayList<Pose2d> ballPoses = new ArrayList<>();
        if (ballTargets == null || ballTargets.isEmpty()) return ballPoses;

        Pose2d robotPose = getInstance().getVisionPose();
        if (robotPose == null) return ballPoses;

        Translation3d camOffset = SubsystemConstants.robotToBallCamera.getTranslation();
        Rotation3d camRotation = SubsystemConstants.robotToBallCamera.getRotation();

        Rotation3d robotHeading = new Rotation3d(0, 0, robotPose.getRotation().getRadians());

        Translation3d camInField = camOffset.rotateBy(robotHeading).plus(new Translation3d(robotPose.getX(), robotPose.getY(), 0));

        for (PhotonTrackedTarget target : ballTargets) {
            if (target == null) continue;

            double yaw   = Math.toRadians(target.getYaw());
            double pitch = Math.toRadians(-target.getPitch());

            Translation3d dCam = new Translation3d(
                Math.cos(pitch) * Math.cos(yaw),
                Math.cos(pitch) * Math.sin(yaw),
                Math.sin(pitch)
            );

            Translation3d dField = dCam.rotateBy(camRotation).rotateBy(robotHeading);

            if (dField.getZ() >= 0) continue;
            
            double tHit = -camInField.getZ() / dField.getZ();
            Logger.recordOutput("Heatmap/tHit", tHit);

            if (tHit < 0) continue;

            double xHit = camInField.getX() + tHit * dField.getX();
            double yHit = camInField.getY() + tHit * dField.getY();

            Logger.recordOutput("Heatmap/xHit", xHit);
            Logger.recordOutput("Heatmap/yHit", yHit);

            if (Double.isNaN(xHit) || Double.isNaN(yHit) || Double.isInfinite(xHit) || Double.isInfinite(yHit)) continue;

            ballPoses.add(new Pose2d(xHit, yHit, Rotation2d.kZero));
        }


        Logger.recordOutput("Heatmap/numPoses", ballPoses.size());
        Logger.recordOutput("Heatmap/ballPose", ballPoses.toArray(new Pose2d[0]));

        return ballPoses;
    }
    public Pose2d getDelayedPose(){
        double phaseDelay = 0.02;
        Pose2d currPose = getVisionPose();
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds,currPose.getRotation());
        return currPose.exp(new Twist2d(speeds.vxMetersPerSecond*phaseDelay,speeds.vyMetersPerSecond*phaseDelay,speeds.omegaRadiansPerSecond*phaseDelay));
    }
    public Pose2d getShootingWhileMovingVirtualPose(){
        return shootingWhileMovingVirtualPose;
    }
    public boolean getShuttling(){
        if(DriverStation.isAutonomousEnabled()) return false;

        double xPos = RobotState.getInstance().getVisionPose().getX();
        boolean inBlue = xPos<FieldConstants.inBlueZoneX&&!Alliance.get();
        boolean inRed = xPos>FieldConstants.inRedZoneX&&Alliance.get();
        Logger.recordOutput("Drive/inBlueZone",inBlue);
        Logger.recordOutput("Drive/inRedZone",inRed);
        return !inBlue&&!inRed;
    }
    public double getWantedShooterAcceleration() {
        return wantedShooterAcceleration;
    }

    public Pose2d getTargetPose() {
        if (!getShuttling()) {
            return (Alliance.get()) ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
        }

        if (getVisionPose().getY() > FieldConstants.yHalfLine) {
            return (Alliance.get()) ? FieldConstants.redShuttlePosUpper : FieldConstants.blueShuttlePosUpper;
        }

        return (Alliance.get()) ? FieldConstants.redShuttlePosLower : FieldConstants.blueShuttlePosLower;
    }

    public void updateShootingWhileMovingVirtualPose() {
        Pose2d currentPose = RobotState.getInstance().getVisionPose();
        Pose2d realTargetPose = getTargetPose();
        Pose2d virtualPose = getTargetPose();
        Pose2d delayedPose = getDelayedPose();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                currentPose.getRotation());

        Translation2d v = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        Translation2d robotToShooter = SubsystemConstants.robotToShooter.getTranslation();
        v = v.plus(
                new Translation2d(
                        robotToShooter.getNorm() * chassisSpeeds.omegaRadiansPerSecond,
                        currentPose.getRotation().rotateBy(robotToShooter.getAngle()).rotateBy(Rotation2d.kCCW_90deg)));

        double vx = v.getX();
        double vy = v.getY();

        for (int i = 0; i < 5; i++) {
            double dx = virtualPose.getX() - delayedPose.getX();
            double dy = virtualPose.getY() - delayedPose.getY();
            double distance = Math.hypot(dx, dy);
           
            double flightTime;
            if(getShuttling()) {
                flightTime = ShooterTreeMap.getInstance().tofTreeMap.getInterpolated(distance);
            } else {
                flightTime = ShooterTreeMap.getInstance().shuttlingTofTreeMap.getInterpolated(distance);
            }

            virtualPose = new Pose2d(
                    realTargetPose.getX() - vx * flightTime,
                    realTargetPose.getY() - vy * flightTime,
                    realTargetPose.getRotation());
        }
        shootingWhileMovingVirtualPose = virtualPose;

        Pose2d shooterPose = delayedPose.transformBy(SubsystemConstants.robotToShooter);
        double distance = virtualPose.getTranslation().getDistance(shooterPose.getTranslation());

        angleToVirtualTarget = Math.atan2(
                virtualPose.getY() - shooterPose.getY(),
                virtualPose.getX() - shooterPose.getX());

        double wantedShooterRPS;
        double wantedHoodRotation;
        if (getShuttling()) {
            wantedShooterRPS = ShooterTreeMap.getInstance().shuttlingRPSTreeMap.getInterpolated(distance);
            wantedHoodRotation = SubsystemConstants.hoodMaxAngleRotations;
        } else {
            wantedShooterRPS = ShooterTreeMap.getInstance().lowerWheelTreeMap.getInterpolated(distance);
            wantedHoodRotation = ShooterTreeMap.getInstance().hoodTreeMap.getInterpolated(distance);
        }

        double currentTimestamp = Timer.getFPGATimestamp();
        if (prevVirtualTargetTimestamp != -1.0) {
            double dt = currentTimestamp - prevVirtualTargetTimestamp;
            if (dt > 0.0) {
                // Drive Auto Aim Angular Velocity
                double deltaAngle = MathUtil.angleModulus(angleToVirtualTarget - prevAngleToVirtualTarget);
                double unfilteredTurretVelocity = deltaAngle / dt;
                virtualTargetAngularVelocity = virtualTargetVelocityFilter.calculate(unfilteredTurretVelocity);

                // Shooter Acceleration
                double deltaRPS = wantedShooterRPS - prevWantedShooterRPS;
                double unfilteredShooterAccel = deltaRPS / dt;
                wantedShooterAcceleration = shooterAccelerationFilter.calculate(unfilteredShooterAccel);

                // Hood Velocity
                double deltaHood = wantedHoodRotation - prevWantedHoodRotation;
                double unfilteredHoodVel = deltaHood / dt;
                wantedHoodVelocity = hoodVelocityFilter.calculate(unfilteredHoodVel);
            }
        } else {
            virtualTargetVelocityFilter.calculate(0.0);
            shooterAccelerationFilter.calculate(0.0);
            hoodVelocityFilter.calculate(0.0);
        }
        prevVirtualTargetTimestamp = currentTimestamp;
        prevAngleToVirtualTarget = angleToVirtualTarget;
        prevWantedShooterRPS = wantedShooterRPS;
        prevWantedHoodRotation = wantedHoodRotation;

        Logger.recordOutput("Drive/speed", Math.hypot(vx, vy));
        Logger.recordOutput("Vision/shootingWhileMovingVirtualPose", shootingWhileMovingVirtualPose);
//        Logger.recordOutput("Vision/angleToVirtualTarget", angleToVirtualTarget);
//        Logger.recordOutput("Vision/virtualTargetAngularVelocity", virtualTargetAngularVelocity);
//        Logger.recordOutput("Vision/wantedShooterAcceleration", wantedShooterAcceleration);
//        Logger.recordOutput("Vision/wantedHoodVelocity", wantedHoodVelocity);
    }

    public double getWantedHoodVelocity() {
        return wantedHoodVelocity;
    }

    public double getVirtualTargetAngularVelocity() {
        return virtualTargetAngularVelocity;
    }

    public double getAngleToRealHub() {
        Pose2d target = (Alliance.get()) ? FieldConstants.redNetPos : FieldConstants.blueNetPos;

        Pose2d shooterPose = getDelayedPose().transformBy(SubsystemConstants.robotToShooter);
        double angle = Math.atan2(
                target.getY() - shooterPose.getY(),
                target.getX() - shooterPose.getX());

        return angle;
    }
}