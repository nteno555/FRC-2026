package frc2025.config;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static Pose2d blueNetPos = new Pose2d(4.626, 4.035, new Rotation2d(0));
    public static Pose2d redNetPos = new Pose2d(11.913, 4.018, new Rotation2d(0));
    public static Pose2d blueShuttlePosUpper = new Pose2d(0.40, 6.50, new Rotation2d(0));
    public static Pose2d blueShuttlePosLower = new Pose2d(0.40, 1.50, new Rotation2d(0));
    public static Pose2d redShuttlePosUpper = new Pose2d(16.00, 6.50, new Rotation2d(0));
    public static Pose2d redShuttlePosLower = new Pose2d(16.00, 1.50, new Rotation2d(0));
    public static double yHalfLine = 4.044;

    public static double blueTrenchX = 4.628;
    public static double redTrenchX = 11.916;
    public static double trenchThreshold = 1.0;
    public static double trenchThresholdAuto = 0.75;
    public static double highBumpY = 5.10;
    public static double lowBumpY = 3.0;

    public static Pose2d blueTrenchHighZone = new Pose2d(3.572, 7.442, Rotation2d.kCW_90deg);
    public static Pose2d blueTrenchHighMid = new Pose2d(6.0, 7.435, Rotation2d.kCW_90deg);
    public static Pose2d blueTrenchLowZone = new Pose2d(3.638, 0.625, Rotation2d.kCCW_90deg);
    public static Pose2d blueTrenchLowMid = new Pose2d(6.0, 0.625, Rotation2d.kCCW_90deg);
    public static Pose2d redTrenchLowZone = new Pose2d(13.041, 7.435, Rotation2d.kCW_90deg);
    public static Pose2d redTrenchLowMid = new Pose2d(10.3, 7.5, Rotation2d.kCW_90deg);
    public static Pose2d redTrenchHighZone = new Pose2d(13.041, 0.625, Rotation2d.kCCW_90deg);
    public static Pose2d redTrenchHighMid = new Pose2d(10.3, 0.55, Rotation2d.kCCW_90deg);
    public static Pose2d[] trenchPoses = {blueTrenchHighZone, blueTrenchHighMid, blueTrenchLowZone, blueTrenchLowMid, redTrenchHighZone, redTrenchHighMid, redTrenchLowZone, redTrenchLowMid};

    public static double highTrenchCutoff = 7.322;
    public static double lowTrenchCutoff = 0.715;

    public static double blueClimbHighY = 4.162;
    public static double blueClimbLowY = 3.282;
    public static double blueClimbX = 1.052;
    public static double redClimbHighY = 4.721;
    public static double redClimbLowY = 3.836;
    public static double redClimbX = 15.513;

    public static double inBlueZoneX = 3.9;
    public static double inRedZoneX = 12.5;
}
