package frc2025;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

import java.util.Optional;


public class RobotState {
    private static RobotState instance;

    private DriverStation.Alliance alliance;

    private VisionPoseEstimator visionPoseEstimator;
    private List<VisionPoseEstimator.TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    private ChassisSpeeds robotRelativeSpeeds;
    private ChassisSpeeds fieldRelativeSpeeds;
    private double pigeonAcceleration;
    private double angleToVirtualTarget;

    private Pose2d shootingWhileMovingVirtualPose;

    private double prevAngleToVirtualTarget = 0.0;
    private double prevVirtualTargetTimestamp = -1.0;
    private double virtualTargetAngularVelocity = 0.0;
    private LinearFilter virtualTargetVelocityFilter = LinearFilter.singlePoleIIR(0.05, 0.02);

    private double turretAngle;
    private double turretVelocity;
    private double prevTurretTimestamp;

    private double turretMt1AngleRadians;
    private boolean turretMt1Valid;

    public RobotState() {
        visionPoseEstimator = new VisionPoseEstimator(new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 1, 1, 0.00001 }));
        turretMt1Valid = false;
        turretMt1AngleRadians = 0;
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

    public Pose2d getDelayedPose() {
        double phaseDelay = 0.03;
        Pose2d currentPose = getVisionPose();
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                currentPose.getRotation()
        );
        return currentPose.exp(
                new Twist2d(
                        speeds.vxMetersPerSecond * phaseDelay,
                        speeds.vyMetersPerSecond * phaseDelay,
                        speeds.omegaRadiansPerSecond * phaseDelay
                )
        );
    }

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
        Pose2d delayedPose = getDelayedPose();

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                currentPose.getRotation()
        );

        Translation2d v = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        Translation2d robotToShooter = SubsystemConstants.robotToShooter.getTranslation();
        v = v.plus(
               new Translation2d(
                       robotToShooter.getNorm() * chassisSpeeds.omegaRadiansPerSecond,
                       currentPose.getRotation().rotateBy(robotToShooter.getAngle()).rotateBy(Rotation2d.kCCW_90deg))
        );

        double vx = v.getX();
        double vy = v.getY();
        double vOmega = chassisSpeeds.omegaRadiansPerSecond;
//        Translation2d shooterFieldRelative = SubsystemConstants.robotToShooter.getTranslation().rotateBy(delayedPose.getRotation());
//        vx += vOmega * shooterFieldRelative.getY();
//        vy += -vOmega * shooterFieldRelative.getX();

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

        double currentTimestamp = Timer.getFPGATimestamp();
        if (prevVirtualTargetTimestamp != -1.0) {
            double dt = currentTimestamp - prevVirtualTargetTimestamp;
            if (dt > 0.0) {
                double deltaAngle = MathUtil.angleModulus(angleToVirtualTarget - prevAngleToVirtualTarget);
                double unfilteredVelocity = deltaAngle / dt;
                virtualTargetAngularVelocity = virtualTargetVelocityFilter.calculate(unfilteredVelocity);
            }
        } else {
            virtualTargetVelocityFilter.calculate(0.0);
        }
        prevVirtualTargetTimestamp = currentTimestamp;
        prevAngleToVirtualTarget = angleToVirtualTarget;

        Logger.recordOutput("Drive/speed", Math.hypot(vx, vy));
        Logger.recordOutput("Vision/shootingWhileMovingVirtualPose", shootingWhileMovingVirtualPose);
        Logger.recordOutput("Vision/angleToVirtualTarget", angleToVirtualTarget);
        Logger.recordOutput("Vision/virtualTargetAngularVelocity", virtualTargetAngularVelocity);
    }

    public double getAngleToRealHub() {
        Pose2d target = (Alliance.get()) ? FieldConstants.redNetPos : FieldConstants.blueNetPos;

        Pose2d shooterPose = getDelayedPose().transformBy(SubsystemConstants.robotToShooter);
        double angle = Math.atan2(
                target.getY() - shooterPose.getY(),
                target.getX() - shooterPose.getX()
        );

        return angle;
    }

    public double getAngleToVirtualTarget() {
        return angleToVirtualTarget;
    }

    public double getVirtualTargetAngularVelocity() {
        return virtualTargetAngularVelocity;
    }

    public void setTurretAngle(double angleRadians) {
//        double now = Utils.getCurrentTimeSeconds();
//        if (prevTurretTimestamp != -1 && (now - prevTurretTimestamp) > 0.01) {
//            return;
//        }
//        prevTurretAngle = turretAngle;
//        prevTurretTimestamp = now;
        turretAngle = angleRadians;
    }

    public void setTurretVelocity(double velocityRadians) {
        this.turretVelocity = velocityRadians;
    }

    public double getTurretVelocity() {
//       if (prevTurretTimestamp == -1 || (Utils.getCurrentTimeSeconds() - prevTurretTimestamp)==0) return 0.0;
//       return (turretAngle - prevTurretAngle)/(Utils.getCurrentTimeSeconds() - prevTurretTimestamp);
//        return turretVelocity;
        return turretVelocity;
    }


    public double getTurretAngle() { return turretAngle; }

    public boolean isHubActive(boolean checkingNear) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        boolean near;

        if (alliance.isEmpty()) return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;
        
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) return true;
        
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {return true;}
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };
        
        if(!checkingNear){
            if (matchTime > 130) return true;
            else if (matchTime > 105) return shift1Active;
            else if (matchTime > 80) return !shift1Active;
            else if (matchTime > 55) return shift1Active;
            else if (matchTime > 30) return !shift1Active;
            else return true;
        }
        else {
            if (matchTime <= 135 && matchTime > 130) near = shift1Active;
            else if (matchTime <= 110 && matchTime > 105) near = !shift1Active;
            else if (matchTime <= 85 && matchTime > 80) near = shift1Active;
            else if (matchTime <= 60 && matchTime > 55) near = !shift1Active;
            else if (matchTime <= 35 && matchTime > 30) near = true;            
            else near = false; 

            if (near){
                double time = (Timer.getFPGATimestamp());
                if (time % 0.5 < 0.25) return true;
                else return false;
            }
            return false;
        }
    }

    public void setTurretMt1AngleRadians(double angle) { turretMt1AngleRadians = angle; }
    public double getTurretMt1AngleRadians() { return turretMt1AngleRadians; }
    public void setTurretMt1Valid(boolean valid) { turretMt1Valid = valid; }
    public boolean getTurretMt1Valid() { return turretMt1Valid; }
}
