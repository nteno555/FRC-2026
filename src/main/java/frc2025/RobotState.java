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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.FieldConstants;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.hood.HoodSubsystem;
import frc2025.subsystems.shooter.ShooterSubsystem;
import frc2025.subsystems.vision.LimelightHelpers;
import frc2025.subsystems.vision.VisionPoseEstimator;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

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

    private List<PhotonTrackedTarget> ballTargets;
    private final Field2d field = new Field2d();
    private ArrayList<String> states;
    private String matchState;

    public RobotState() {
        visionPoseEstimator = new VisionPoseEstimator(new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 1, 1, 0.00001 }));
        shuttling = false;
        prevTurretTimestamp = -1;
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
        if (!shuttling) {
            return (Alliance.get()) ? FieldConstants.redNetPos : FieldConstants.blueNetPos;
        }

        if (visionPoseEstimator.getLatestPose().getY() > FieldConstants.yHalfLine) {
            return (Alliance.get()) ? FieldConstants.redShuttlePosUpper : FieldConstants.blueShuttlePosUpper;
        }

        return (Alliance.get()) ? FieldConstants.redShuttlePosLower : FieldConstants.blueShuttlePosLower;
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

//        if (SubsystemConstants.hasTurret) {
            Translation2d shooterFieldRelative = SubsystemConstants.robotToShooter.getTranslation().rotateBy(delayedPose.getRotation());

            vx += -vOmega * shooterFieldRelative.getY();
            vy += vOmega * shooterFieldRelative.getX();
//        }

        for (int i = 0; i < 5; i++) {
            double dx = virtualPose.getX() - delayedPose.getX();
            double dy = virtualPose.getY() - delayedPose.getY();
            double distance = Math.hypot(dx, dy);

//            double flightTime = ShooterTreeMap.getInstance().tofTreeMap.getInterpolated(distance);
            double flightTime = 0.75;
            //ShooterTreeMap.getInstance().getRealTof(distance);

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

    public void setWantAutoShoot(boolean want) {wantAutoShoot = want;}
    public boolean getWantAutoShoot() {return wantAutoShoot;}

    public void setTurretAngle(double angleRadians) {
        prevTurretAngle = turretAngle;
        prevTurretTimestamp = Utils.getCurrentTimeSeconds();
        turretAngle = angleRadians;
    }

    public void setTurretVelocity(double vel) {
        //turretVelocity = vel;
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

    public List<PhotonTrackedTarget> getBallTargets() {
        return this.ballTargets;
    }

    public void setBallTargets(List<PhotonTrackedTarget> targets) {
        this.ballTargets = targets;
    }

    // public ArrayList<Pose2d> getBallFieldPoses(){
    //     ArrayList<Pose2d> ballPoses = new ArrayList<Pose2d>();
    //     Pose2d robotPose = getInstance().getVisionPose();
    //     if (ballTargets == null) return null;
    //     for (PhotonTrackedTarget t : ballTargets) {
    //         double distanceToTarget = PhotonUtils.calculateDistanceToTargetMeters(
    //             SubsystemConstants.photonCameraHeight, 
    //             0.75, 
    //             SubsystemConstants.robotToPhotonCameraPitch,
    //             Math.toRadians(t.getPitch())
    //         );

    //         Translation2d cameraToBall = new Translation2d(distanceToTarget, Rotation2d.fromDegrees(t.getYaw()));

    //         ballPoses.add(robotPose.transformBy(new Transform2d(cameraToBall,Rotation2d.kZero)));
    //     }
    //     return ballPoses;
    // }

    public ArrayList<Pose2d> getBallFieldPoses() {
        ArrayList<Pose2d> ballPoses = new ArrayList<>();
        Pose2d robotPose = getVisionPose();

        if (ballTargets == null) return ballPoses;

        Translation3d camTranslation = SubsystemConstants.robotToPhotonCamera.getTranslation();
        Rotation3d camRotation = SubsystemConstants.robotToPhotonCamera.getRotation();

        double camX = camTranslation.getX();
        double camY = camTranslation.getY();
        double camZ = camTranslation.getZ();

        for (PhotonTrackedTarget t : ballTargets) {
            double yaw = Math.toRadians(t.getYaw());
            double pitch = Math.toRadians(t.getPitch());

            double xCam = Math.cos(pitch) * Math.cos(yaw);
            double yCam = Math.cos(pitch) * Math.sin(yaw);
            double zCam = Math.sin(pitch);

            Translation3d dCam = new Translation3d(xCam, yCam, zCam);
            Translation3d dRobot = dCam.rotateBy(camRotation);
            if (dRobot.getZ() >= 0) continue;
            double tHit = (-camZ / dRobot.getZ());
            double xHit = camX + (tHit * dRobot.getX());
            double yHit = camY + (tHit * dRobot.getY());

            Translation2d robotToBall = new Translation2d(xHit, yHit);

            Pose2d ballPose = robotPose.transformBy(new Transform2d(robotToBall, Rotation2d.kZero));

            ballPoses.add(ballPose);
        }

        return ballPoses;
    }

    public Pose2d bestClusterPose(){
        ArrayList<Pose2d> ballPoses = getBallFieldPoses();
        Pose2d robotPose = getVisionPose();   
        int bestIndex = -1;
        int bestNumNeighbors = 0;
        double epsilon = 0.5;        
        
        if (ballPoses.size()<1) return null;

        for (int i=0; i < ballPoses.size(); i++){
            int ballsInRange=0;
            for (int j = 0; j < ballPoses.size(); j++){
                if(j==i) continue;
                if (ballPoses.get(i).getTranslation().getDistance(ballPoses.get(j).getTranslation()) <= epsilon) {
                    ballsInRange++;
                }
            }
            double distance = robotPose.getTranslation().getDistance(ballPoses.get(i).getTranslation());
            ballsInRange += 1/distance;

            if(ballsInRange > bestNumNeighbors) {
                bestIndex = i;
                bestNumNeighbors = ballsInRange;
            }
        }
        if (bestIndex == -1) { 
            Pose2d closestBall = ballPoses.get(0);
            double closestDistance = robotPose.getTranslation().getDistance(closestBall.getTranslation());
            for (int i = 1; i < ballPoses.size(); i++) {
                double distance = robotPose.getTranslation().getDistance(ballPoses.get(i).getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestBall = ballPoses.get(i);
                }
            }
            Logger.recordOutput("DriverStation/clusterPose", closestBall);
            return closestBall;
        }
        Logger.recordOutput("DriverStation/clusterPose", ballPoses.get(bestIndex));
        return ballPoses.get(bestIndex);
    }

    public void setFieldPoses(){
        if(getBallFieldPoses() != null) {
            field.getObject("Fuel").setPoses(getBallFieldPoses());   
        }
        field.setRobotPose(getVisionPose());
    }

    public Field2d getField(){
        return field; 
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

    /*public void updateStates(){
        Logger.recordOutput("DriverStation/thehub", isHubActive());
        //hub
        if(isHubActive()) addState("HUB_ACTIVE"); 
        // else removeState("HUB_ACTIVE"); 

        //Shuttling
        Logger.recordOutput("DriverStation/shuttling", shuttling);
        if (shuttling) addState("SHUTTLING");
        // else if (!shuttling) removeState("SHUTTLING");
        
        //Autoshooing
        if(wantAutoShoot) addState("AUTO_SHOOTING");
        // else removeState("AUTO_SHOOTING");

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
    }*/
}
