package frc2025.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SubsystemConstants {
    public static double armkS = 0;
    public static double armkV = 0.13;
    public static double armkP = 0.9;
    public static double armkI = 0.0;
    public static double armkD = 0.05;
    public static double armCruiseVelocity = 100;
    public static double armAcceleration = 200;
    public static double armJerk = 7000;
    public static double armSupplyCurrentLimit = 40;
    public static double armStatorCurrentLimit = 40;
    public static double armPeakForwardTorqueCurrent = 40;
    public static double armPeakReverseTorqueCurrent = 40;
    public static double armTorqueNeutralDeadband = 1;
    public static double armRotationsPerDegree = 0.208333333333;
    public static double armkGFactor = 0.30;
    public static double armZeroOffsetDegrees = 0;
    public static double armL4Pos = -8.5209;
    public static double armL3Pos = -10.5122;
    public static double armL2Pos = -13.1709;
    public static double armRemoveAlgaePos = -32.448;
    public static double armProcessorPos = -39.7646;
    public static double armBargePos = -10.4360;
    public static double armHandoffPos = -36;

    public static double elevatorkS = 0.145;
    public static double elevatorkV = 0.12269;
    public static double elevatorkA = 0;
    public static double elevatorkG = 0.308;
    public static double elevatorkP = 0.01;
    public static double elevatorkI = 0.0;
    public static double elevatorkD = 0.0;
<<<<<<< Updated upstream
    public static double elevatorCruiseVelocity = 800;
    public static double elevatorAcceleration = 250;
=======
    // Loaded Constants
    public static double elevatorkSloaded = 0.155;
    public static double elevatorkVloaded = 0.128;
    public static double elevatorkAloaded = 0.0;
    public static double elevatorkGloaded = 0.340;
    public static double elevatorkPloaded = 0.01;
    public static double elevatorkIloaded = 0.0;
    public static double elevatorkDloaded = 0.0;

    public static double elevatorCruiseVelocity = 1000;
    public static double elevatorAcceleration = 300;
>>>>>>> Stashed changes
    public static double elevatorJerk = 100000;
    public static double elevatorSupplyCurrentLimit = 40;
    public static double elevatorStatorCurrentLimit = 100;
    public static double elevatorPeakForwardTorqueCurrent = 40;
    public static double elevatorPeakReverseTorqueCurrent = 40;
    public static double elevatorTorqueNeutralDeadband = 1;
    public static double elevatorL4Pos = 15.0;
    public static double elevatorL3Pos = 1.93862;
    public static double elevatorL2Pos = 0;
    public static double elevatorL3AlgaePos = 14.03;
    public static double elevatorL2AlgaePos = 4.5;
    public static double elevatorBargePos = 19.5813;
    public static double elevatorProcessorPos = 0;
    public static double elevatorHandoffPos = 0.5;

}
