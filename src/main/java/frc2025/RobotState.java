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

import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

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

    private boolean shuttling;
    private Pose2d shootingWhileMovingVirtualPose;

    private boolean wantAutoShoot;

    private double turretAngle;
    private double prevTurretAngle;
    private double prevTurretTimestamp;

    private ArrayList<String> states;
    private String matchState;

    public RobotState() {
        visionPoseEstimator = new VisionPoseEstimator(new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 1, 1, 0.00001 }));
        shuttling = false;
        prevTurretTimestamp = -1;
        states = new ArrayList<String>();
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

        boolean inBlueZone = xPos < FieldConstants.inBlueZoneX && !Alliance.get();
        boolean inRedZone = xPos > FieldConstants.inRedZoneX && Alliance.get();

        Logger.recordOutput("Shooter/inBlueZone", inBlueZone);
        Logger.recordOutput("Shooter/inRedZone", inRedZone);

        if (inBlueZone || inRedZone) shuttling = false;
        else shuttling = true;
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
        Pose2d delayedPose = currentPose.exp(
                new Twist2d(vx * phaseDelay, vy * phaseDelay, vOmega * phaseDelay)
        );

        Translation2d shooterFieldRelative = SubsystemConstants.robotToShooter.getTranslation().rotateBy(delayedPose.getRotation());

        vx += -vOmega * shooterFieldRelative.getY();
        vy += vOmega * shooterFieldRelative.getX();

        for (int i = 0; i < 20; i++) {
            double dx = virtualPose.getX() - delayedPose.getX();
            double dy = virtualPose.getY() - delayedPose.getY();
            double distance = Math.hypot(dx, dy);

            double flightTime = ShooterTreeMap.getInstance().tofTreeMap.getInterpolated(distance);

            virtualPose = new Pose2d(
                    realTargetPose.getX() + vx * flightTime,
                    realTargetPose.getY() + vy * flightTime,
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

    public boolean isHubActive() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
            // If we have invalid game data, assume hub is active.
            return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            return true;
        }
    }

    public void updateStates(){
        Logger.recordOutput("DriverStation/thehub", isHubActive());
        //hub
        if(isHubActive()) addState("HUB_ACTIVE"); 
        else removeState("HUB_ACTIVE"); 

        //Shuttling
        Logger.recordOutput("DriverStation/shuttling", shuttling);
        if (shuttling) addState("SHUTTLING");
        else if (!shuttling) removeState("SHUTTLING");
        
        //Autoshooing
        if(wantAutoShoot) addState("AUTO_SHOOTING");
        else removeState("AUTO_SHOOTING");

    }

    public void addState(String state) {
        if(!states.contains(state)){
            states.add(state);
        }
    }

    public void removeState(String state) {
        states.remove(state);
    }

    public ArrayList<String> getStates() {
        return states;
    }

    public void updateMatchState(){
        double matchTime = DriverStation.getMatchTime();
        if(DriverStation.isAutonomous()) matchState = "AUTONOMOUS";
        if(DriverStation.isTeleop() && matchTime < 130) matchState = "TELEOP";
        if(matchTime > 130) matchState = "ENDGAME";
    }

    public String getMatchState(){
        return matchState;
    }
}
