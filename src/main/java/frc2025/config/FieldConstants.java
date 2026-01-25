package frc2025.config;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static Pose2d blueNetPos = new Pose2d(4.631, 4.044, new Rotation2d(0));
    public static Pose2d redNetPos = new Pose2d(11.913, 4.018, new Rotation2d(0));
    public static Pose2d blueShuttlePosUpper = new Pose2d(2.477, 6.50, new Rotation2d(0));
    public static Pose2d blueShuttlePosLower = new Pose2d(2.477, 1.944, new Rotation2d(0));
    public static Pose2d redShuttlePosUpper = new Pose2d(13.995, 6.322, new Rotation2d(0));
    public static Pose2d redShuttlePosLower = new Pose2d(13.995, 1.944, new Rotation2d(0));
    public static double yHalfLine = 4.044;
}
