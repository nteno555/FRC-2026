package frc2025.config;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static Pose2d blueNetPos = new Pose2d(4.631, 4.044, new Rotation2d(0));
    public static Pose2d redNetPos = new Pose2d(11.913, 4.018, new Rotation2d(0));
    public static Pose2d blueShuttlePosUpper = new Pose2d(2.477, 6.50, new Rotation2d(0));
    public static Pose2d blueShuttlePosLower = new Pose2d(2.477, 1.50, new Rotation2d(0));
    public static Pose2d redShuttlePosUpper = new Pose2d(13.995, 6.50, new Rotation2d(0));
    public static Pose2d redShuttlePosLower = new Pose2d(13.995, 1.50, new Rotation2d(0));
    public static double yHalfLine = 4.044;

    public static double blueTrenchX = 4.628;
    public static double redTrenchX = 11.916;
    public static double trenchThreshold = 1.0;
    public static double trenchThresholdAuto = 0.75;
    public static double highBumpY = 5.10;
    public static double lowBumpY = 3.0;

    public static Pose2d blueTrenchHighZone = new Pose2d(3.422, 7.435, Rotation2d.kCW_90deg);
    public static Pose2d blueTrenchHighMid = new Pose2d(5.592, 7.435, Rotation2d.kCW_90deg);
    public static Pose2d blueTrenchLowZone = new Pose2d(3.422, 0.741, Rotation2d.kCCW_90deg);
    public static Pose2d blueTrenchLowMid = new Pose2d(5.592, 0.741, Rotation2d.kCCW_90deg);
    public static Pose2d redTrenchHighZone = new Pose2d(13.041, 7.435, Rotation2d.kCW_90deg);
    public static Pose2d redTrenchHighMid = new Pose2d(10.924, 7.435, Rotation2d.kCW_90deg);
    public static Pose2d redTrenchLowZone = new Pose2d(13.041, 0.741, Rotation2d.kCCW_90deg);
    public static Pose2d redTrenchLowMid = new Pose2d(10.924, 0.741, Rotation2d.kCCW_90deg);
    public static Pose2d[] trenchPoses = {blueTrenchHighZone, blueTrenchHighMid, blueTrenchLowZone, blueTrenchLowMid, redTrenchHighZone, redTrenchHighMid, redTrenchLowZone, redTrenchLowMid};

    public static double inBlueZoneX = 3.7;
    public static double inRedZoneX = 12.8;
}
