package frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.*;
import frc2025.RobotState;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class BallPoseEstimator {
    private static final double ballDiameter = 0.0254;
    private static final double maxDist = 5.0;
    private static final double maxReprojError = 8.0;

    private final PhotonCamera camera;
    private final Transform3d  robotToCamera;
    private Mat cameraMatrix;
    private MatOfDouble distCoeffs;
    private boolean calibrated = false;

    private final MatOfPoint3f objectPoints;

    public BallPoseEstimator(String cameraName, Transform3d robotToCamera) {
        this.camera = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;

        double r = (ballDiameter / 2.0);
        objectPoints = new MatOfPoint3f(new Point3(-r, -r, 0), new Point3( r, -r, 0), new Point3( r,  r, 0), new Point3(-r,  r, 0));
    }

    public ArrayList<Pose2d> getBallFieldPoses(Pose2d robotPose) {
        if (!calibrated) loadCalibration();
        if (!calibrated) return null;
        List<PhotonTrackedTarget> ballTargets = RobotState.getInstance().getBallTargets();
        ArrayList<Pose2d> ballPoses = new ArrayList<>();
        if (ballTargets.isEmpty() || ballTargets == null) {return null;}
        for (PhotonTrackedTarget target : ballTargets) {
            
            List<TargetCorner> corners = target.getMinAreaRectCorners();
            if (corners == null || corners.size() < 4) continue;

            MatOfPoint2f imagePoints = new MatOfPoint2f(new Point(corners.get(0).x, corners.get(0).y), new Point(corners.get(1).x, corners.get(1).y), new Point(corners.get(2).x, corners.get(2).y), new Point(corners.get(3).x, corners.get(3).y));

            Mat rvec = new Mat();
            Mat tvec = new Mat();
            boolean solved = Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false, Calib3d.SOLVEPNP_IPPE_SQUARE);

            if (!solved) continue;

            MatOfPoint2f reprojected = new MatOfPoint2f();
            Calib3d.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, reprojected);
            if (reprojectionError(imagePoints, reprojected) > maxReprojError) continue;

            double[] t = tvec.get(0, 0);
            Translation3d cameraToball = new Translation3d(t[2], -t[0], -t[1]);

            double dist = cameraToball.getNorm();
            if (dist <= 0 || dist > maxDist) continue;

            Pose3d fieldToRobot = new Pose3d(new Translation3d(robotPose.getX(), robotPose.getY(), 0), new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
            Pose3d fieldToCamera = fieldToRobot.transformBy(robotToCamera);

            Translation3d ballOnField = fieldToCamera.getTranslation().plus(cameraToball.rotateBy(fieldToCamera.getRotation()));
            ballPoses.add(new Pose2d(ballOnField.getX(), ballOnField.getY(), Rotation2d.kZero));
        }
        return ballPoses;

    }

    private double reprojectionError(MatOfPoint2f a, MatOfPoint2f b) {
        return (Core.norm(a, b, Core.NORM_L2)) / (Math.sqrt(a.rows()));
    }

    private void loadCalibration() {
        var cal  = camera.getCameraMatrix();
        var dist = camera.getDistCoeffs();
        if (cal.isEmpty() || dist.isEmpty()) return;

        var cm = cal.get();
        cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                cameraMatrix.put(r, c, cm.get(r, c));
            }
        }
        var dc = dist.get();
        distCoeffs = new MatOfDouble(dc.getNumRows(), 1, CvType.CV_64FC1);
        for (int i = 0; i < dc.getNumRows(); i++) {
            distCoeffs.put(i, 0, dc.get(i, 0));
        }

        calibrated = true;
    }
}