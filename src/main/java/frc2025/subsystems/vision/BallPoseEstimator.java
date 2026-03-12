package frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import frc2025.RobotState;

import org.littletonrobotics.junction.Logger;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class BallPoseEstimator {
    private static final double ballDiameter = 0.150114;
    private static final double maxDist = 5.0;
    private static final double maxReprojError = 8.0;

    private final Transform3d  robotToCamera;
    private Mat cameraMatrix;
    private MatOfDouble distCoeffs;
    private boolean calibrated = false;

    private final MatOfPoint3f objectPoints;

    public BallPoseEstimator(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;

        double r = (ballDiameter / 2.0);
        objectPoints = new MatOfPoint3f(new Point3(-r, -r, 0), new Point3( r, -r, 0), new Point3( r,  r, 0), new Point3(-r,  r, 0));
    }

    public ArrayList<Pose2d> getBallFieldPoses(Pose2d robotPose) {
        ArrayList<Pose2d> ballPoses = new ArrayList<>();

        if (robotPose == null) return ballPoses;

        if (!calibrated) loadCalibration();
        if (!calibrated) return ballPoses;

        List<PhotonTrackedTarget> ballTargets = RobotState.getInstance().getBallTargets();
        if (ballTargets == null || ballTargets.isEmpty()) return ballPoses;

        for (PhotonTrackedTarget target : ballTargets) {
            if (target == null) continue;

            List<TargetCorner> corners = target.getMinAreaRectCorners();
            if (corners == null || corners.size() < 4) continue;

            MatOfPoint2f imagePoints = new MatOfPoint2f(
                    new Point(corners.get(0).x, corners.get(0).y),
                    new Point(corners.get(1).x, corners.get(1).y),
                    new Point(corners.get(2).x, corners.get(2).y),
                    new Point(corners.get(3).x, corners.get(3).y)
            );

            Mat rvec = new Mat();
            Mat tvec = new Mat();
            boolean solved = Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, Calib3d.SOLVEPNP_IPPE_SQUARE);
            if (!solved || tvec.empty()) continue;

            MatOfPoint2f reprojected = new MatOfPoint2f();
            Calib3d.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, reprojected);
            if (reprojectionError(imagePoints, reprojected) > maxReprojError) continue;

            double[] t = tvec.get(0, 0);
            if (t == null || t.length < 3) continue;

            Translation3d cameraToball = new Translation3d(t[2], -t[0], -t[1]);
            double dist = cameraToball.getNorm();
            if (dist <= 0 || dist > maxDist) continue;

            Pose3d fieldToRobot = new Pose3d(
                    new Translation3d(robotPose.getX(), robotPose.getY(), 0),
                    new Rotation3d(0, 0, robotPose.getRotation().getRadians())
            );
            Pose3d fieldToCamera = fieldToRobot.transformBy(robotToCamera);

            Translation3d ballOnField = fieldToCamera.getTranslation().plus(cameraToball.rotateBy(fieldToCamera.getRotation()));
            ballPoses.add(new Pose2d(ballOnField.getX(), ballOnField.getY(), Rotation2d.kZero));
        }

        if (!ballPoses.isEmpty()) {
            Logger.recordOutput("Heatmap/ballFieldPoses", ballPoses.get(0));
        }

        return ballPoses;
    }

    private double reprojectionError(MatOfPoint2f a, MatOfPoint2f b) {
        return (Core.norm(a, b, Core.NORM_L2)) / (Math.sqrt(a.rows()));
    }

    private void loadCalibration() {

        cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
        cameraMatrix.put(0, 0, 366.21);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, 319.02);
        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, 366.11);
        cameraMatrix.put(1, 2, 232.73);
        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);

        distCoeffs = new MatOfDouble(-0.311, 0.139, 0.001, 0.0, 0.051, 0.06, -0.064, 0.148);

        calibrated = true;
    }
}