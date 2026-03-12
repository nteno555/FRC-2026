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
import frc2025.subsystems.vision.BallPoseEstimator;
import frc2025.subsystems.vision.LimelightHelpers;
import frc2025.subsystems.vision.VisionPoseEstimator;
import frc2025.util.Alliance;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
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
    BallPoseEstimator ballPoseEstimator = new BallPoseEstimator(SubsystemConstants.robotToPhotonCamera);

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
    // public ArrayList<Pose2d> getBallFieldPoses() {
    //     ArrayList<Pose2d> ballFieldPoses = new ArrayList<>();
    //     if (ballTargets == null) {return ballFieldPoses;}

    //     Pose2d robotPose = visionPoseEstimator.getLatestPose();

    //     for (PhotonTrackedTarget target : ballTargets) {
    //         Transform3d cameraToTarget = target.getBestCameraToTarget();
    //         Transform3d robotToCamera = SubsystemConstants.robotToPhotonCamera;
    //         if (cameraToTarget != null) {
    //             Logger.recordOutput("Heatmap/cameraToTarget", cameraToTarget);
    //         }
    //         double fieldX = robotPose.getX() + robotToCamera.getX() + cameraToTarget.getX();
    //         double fieldY = robotPose.getY() + robotToCamera.getY() + cameraToTarget.getY();
    //         double fieldRotation = robotPose.getRotation().getRadians()
    //                             + robotToCamera.getRotation().getZ()
    //                             + cameraToTarget.getRotation().getZ();

    //         Pose2d targetFieldPose2d = new Pose2d(fieldX, fieldY, new Rotation2d(fieldRotation));
    //         ballFieldPoses.add(targetFieldPose2d);
    //     }
    //     if (ballFieldPoses != null || !ballFieldPoses.isEmpty()) {
    //         Logger.recordOutput("Heatmap/ballFieldPoses", ballFieldPoses.get(0));
    //     }
    //     return ballFieldPoses;
    // }

    // public ArrayList<Pose2d> getBallFieldPoses() {

    //     ArrayList<Pose2d> ballPoses = new ArrayList<>();
    //     if (ballTargets == null) return ballPoses;

    //     Pose2d robotPose = getVisionPose();

    //     Translation3d camTranslation = SubsystemConstants.robotToPhotonCamera.getTranslation();
    //     Rotation3d camRotation = SubsystemConstants.robotToPhotonCamera.getRotation();

    //     double camX = camTranslation.getX();
    //     double camY = camTranslation.getY();
    //     double camZ = camTranslation.getZ();

    //     for (PhotonTrackedTarget target : ballTargets) {

    //         double yaw = Math.toRadians(target.getYaw());
    //         double pitch = Math.toRadians(target.getPitch());

    //         double x = Math.cos(pitch) * Math.cos(yaw);
    //         double y = Math.cos(pitch) * Math.sin(yaw);
    //         double z = Math.sin(pitch);

    //         Translation3d rayCam = new Translation3d(x, y, z);

    //         Translation3d rayRobot = rayCam.rotateBy(camRotation);

    //         if (Math.abs(rayRobot.getZ()) < 1e-6) continue;

    //         double t = -camZ / rayRobot.getZ();

    //         if (t < 0) continue;

    //         double hitX = camX + t * rayRobot.getX();
    //         double hitY = camY + t * rayRobot.getY();

    //         Translation2d robotToBall = new Translation2d(hitX, hitY);

    //         Pose2d ballPose =
    //             robotPose.transformBy(new Transform2d(robotToBall, Rotation2d.kZero));
            
    //         ballPoses.add(ballPose);
    //     }


    //     return ballPoses;
    // }
    // public ArrayList<Pose2d> getBallFieldPoses() {
    //     ArrayList<Pose2d> ballFieldPoses = new ArrayList<>();

    //     Pose2d robotPose = visionPoseEstimator.getLatestPose();
    //     if (robotPose == null) return ballFieldPoses; 

    //     List<PhotonTrackedTarget> targets = RobotState.getInstance().getBallTargets();
    //     if (targets == null || targets.isEmpty()) return ballFieldPoses;

    //     if (SubsystemConstants.robotToPhotonCamera == null) return ballFieldPoses;

    //     if (ShooterTreeMap.getInstance() == null ||
    //         ShooterTreeMap.getInstance().areaToDistanceTreeMap == null) return ballFieldPoses; 

    //     for (PhotonTrackedTarget target : targets) {
    //         if (target == null) continue;

    //         double yawRad = -Math.toRadians(target.getYaw());

    //         double distance = ShooterTreeMap.getInstance().areaToDistanceTreeMap.getInterpolated(target.getArea());
    //         if (Double.isNaN(distance) || Double.isInfinite(distance)) continue;

    //         double xCamera = distance * Math.cos(yawRad);
    //         double yCamera = -distance * Math.sin(yawRad);
    //         Logger.recordOutput("Heatmap/distance", distance);
    //         Pose2d cameraRelative = new Pose2d(xCamera, yCamera, new Rotation2d());

    //         Transform2d cameraToRobot = new Transform2d(
    //             new Translation2d(SubsystemConstants.robotToPhotonCamera.getX(),
    //                             SubsystemConstants.robotToPhotonCamera.getY()),
    //             new Rotation2d(SubsystemConstants.robotToPhotonCamera.getRotation().getZ())
    //         );

    //         Pose2d robotRelative = cameraRelative.plus(cameraToRobot);

    //         Pose2d fieldPose = robotPose.plus(new Transform2d(
    //             robotRelative.getTranslation(),
    //             robotRelative.getRotation()
    //         ));

    //         ballFieldPoses.add(fieldPose);
    //     }
    //     if (ballFieldPoses != null || !ballFieldPoses.isEmpty()) {
    //         Logger.recordOutput("Heatmap/ballFieldPose", ballFieldPoses.get(0));
    //     }
    //     return ballFieldPoses;
    // }
    // public ArrayList<Pose2d> getBallFieldPoses() {
    //     ArrayList<Pose2d> fieldPoses = new ArrayList<>();
    //     double objectHeightMeters = 0.150114; // ball height
    //     int count = 0;


    //     if (visionPoseEstimator == null || visionPoseEstimator.getLatestPose() == null) {
    //         return fieldPoses;
    //     }
    //     Pose2d visionPose = visionPoseEstimator.getLatestPose();

    //     Transform3d robotToPhotonCamera = SubsystemConstants.robotToPhotonCamera;
    //     if (robotToPhotonCamera == null) return fieldPoses;

    //     if (ballTargets == null || ballTargets.isEmpty()) return fieldPoses;


    //     double fx = 366.21;
    //     double fy = 366.11;
    //     double cx = 319.02;
    //     double cy = 232.73;

    //     for (PhotonTrackedTarget target : ballTargets) {
    //         if (target == null) continue;

    //         var corners = target.getMinAreaRectCorners();
    //         if (corners == null || corners.size() < 4) continue;

    //         double sumX = 0, sumY = 0;
    //         for (var c : corners) {
    //             if (c == null) continue;
    //             sumX += c.x;
    //             sumY += c.y;
    //         }
    //         double u = sumX / corners.size();
    //         double v = sumY / corners.size();


    //         double xNorm = (u - cx) / fx;
    //         double yNorm = (cy - v) / fy;

    //         double cameraHeight = robotToPhotonCamera.getTranslation().getZ();
    //         double deltaH = objectHeightMeters - cameraHeight;

    //         double rayPitch = Math.atan2(yNorm, 1.0);
    //         if (Math.abs(Math.tan(rayPitch)) < 1e-6) continue;

    //         double zCam = deltaH / Math.tan(rayPitch);
    //         double xCam = xNorm * zCam; 

    //         double camYaw = robotToPhotonCamera.getRotation().getZ();
    //         double cosYaw = Math.cos(camYaw);
    //         double sinYaw = Math.sin(camYaw);

    //         double xRobot = zCam * cosYaw - xCam * sinYaw + robotToPhotonCamera.getTranslation().getX();
    //         double yRobot = zCam * sinYaw + xCam * cosYaw + robotToPhotonCamera.getTranslation().getY();

    //         double robotCos = Math.cos(visionPose.getRotation().getRadians());
    //         double robotSin = Math.sin(visionPose.getRotation().getRadians());
    //         double xField = visionPose.getX() + (xRobot * robotCos - yRobot * robotSin);
    //         double yField = visionPose.getY() + (xRobot * robotSin + yRobot * robotCos);

    //         Pose2d ballFieldPose = new Pose2d(xField, yField, new Rotation2d());

    //         fieldPoses.add(ballFieldPose);
    //         Logger.recordOutput("Heatmap/ballFieldPose" + count, ballFieldPose);
    //         count++;
    //     }

    //     if (!fieldPoses.isEmpty()) {
    //         Logger.recordOutput("Heatmap/ballFieldPoses", fieldPoses.get(0));
    //     }

    //     return fieldPoses;
    // }

    public Pose2d bestClusterPose(){
        ArrayList<Pose2d> ballPoses = ballPoseEstimator.getBallFieldPoses(visionPoseEstimator.getLatestPose());
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
        if(ballPoseEstimator.getBallFieldPoses(visionPoseEstimator.getLatestPose()) != null) {
            field.getObject("Fuel").setPoses(ballPoseEstimator.getBallFieldPoses(visionPoseEstimator.getLatestPose()));   
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
