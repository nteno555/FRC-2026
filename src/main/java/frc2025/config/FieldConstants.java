package frc2025.config;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static Map<Integer, Pose2d> intakePositions = new HashMap<>(
            Map.of(
                    0, new Pose2d(1.451, 7.373, Rotation2d.fromDegrees(-55)),
                    1, new Pose2d(1.425, 0.625, Rotation2d.fromDegrees(55))
            )
    );

    public static Map<Integer, Double> xScoreOffsetsBlue = new HashMap<>(
            Map.ofEntries(
                    Map.entry(0, 0.17),
                    Map.entry(1, -0.17),
                    Map.entry(2, 0.17),
                    Map.entry(3, -0.17),
                    Map.entry(4, 0.17),
                    Map.entry(5, -0.17),
                    Map.entry(6, 0.17),
                    Map.entry(7, -0.17),
                    Map.entry(8, 0.17),
                    Map.entry(9, -0.17),
                    Map.entry(10, 0.17),
                    Map.entry(11, -0.17)
            )
    );

    public static Map<Integer, Double> yScoreOffsetsBlue = new HashMap<>(
            Map.ofEntries(
                    Map.entry(0, 0.95),
                    Map.entry(1, 0.95),
                    Map.entry(2, 0.95),
                    Map.entry(3, 0.95),
                    Map.entry(4, 0.95),
                    Map.entry(5, 0.95),
                    Map.entry(6, 0.95),
                    Map.entry(7, 0.95),
                    Map.entry(8, 0.95),
                    Map.entry(9, 0.95),
                    Map.entry(10, 0.95),
                    Map.entry(11, 0.95)
            )
    );

    public static Map<Integer, Double> yScoreOffsetsBlueL2 = new HashMap<>(
            Map.ofEntries(
                    Map.entry(0, 0.72),
                    Map.entry(1, 0.72),
                    Map.entry(2, 0.72),
                    Map.entry(3, 0.72),
                    Map.entry(4, 0.72),
                    Map.entry(5, 0.72),
                    Map.entry(6, 0.72),
                    Map.entry(7, 0.72),
                    Map.entry(8, 0.72),
                    Map.entry(9, 0.72),
                    Map.entry(10, 0.72),
                    Map.entry(11, 0.72)
            )
    );

    public static Map<Integer, Double> yScoreOffsetsRed = new HashMap<>(
            Map.ofEntries(
                    Map.entry(0, 0.95),
                    Map.entry(1, 0.95),
                    Map.entry(2, 0.95),
                    Map.entry(3, 0.95),
                    Map.entry(4, 0.95),
                    Map.entry(5, 0.95),
                    Map.entry(6, 0.95),
                    Map.entry(7, 0.95),
                    Map.entry(8, 0.95),
                    Map.entry(9, 0.95),
                    Map.entry(10, 0.95),
                    Map.entry(11, 0.95),
                    Map.entry(12, 0.95)
            )
    );

        public static Map<Integer, Double> yScoreOffsetsRedL2 = new HashMap<>(
            Map.ofEntries(
                    Map.entry(0, 0.72),
                    Map.entry(1, 0.72),
                    Map.entry(2, 0.72),
                    Map.entry(3, 0.72),
                    Map.entry(4, 0.72),
                    Map.entry(5, 0.72),
                    Map.entry(6, 0.72),
                    Map.entry(7, 0.72),
                    Map.entry(8, 0.72),
                    Map.entry(9, 0.72),
                    Map.entry(10, 0.72),
                    Map.entry(11, 0.72)
            )
    );

    public static Map<Integer, Double> xScoreOffsetsRed = new HashMap<>(
            Map.ofEntries(
                    Map.entry(0, 0.17),
                    Map.entry(1, -0.17),
                    Map.entry(2, 0.17),
                    Map.entry(3, -0.17),
                    Map.entry(4, 0.17),
                    Map.entry(5, -0.17),
                    Map.entry(6, 0.17),
                    Map.entry(7, -0.17),
                    Map.entry(8, 0.17),
                    Map.entry(9, -0.17),
                    Map.entry(10, 0.17),
                    Map.entry(11, -0.17)
            )
    );

    public static Pose2d blueNetPos = new Pose2d(8.263, 5.904, new Rotation2d(0));
    public static Pose2d redNetPos = new Pose2d(9.360, 2.033, new Rotation2d(0));
    public static Pose2d blueProcessorPos = new Pose2d(6.338, 0.552, Rotation2d.fromDegrees(-90));
    public static Pose2d redProcessorPos = new Pose2d(11.586, 7.517, Rotation2d.fromDegrees(90));
}
