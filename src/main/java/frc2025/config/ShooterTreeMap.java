package frc2025.config;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.subsystems.hood.HoodSubsystem;
import frc2025.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTreeMap {
    private static ShooterTreeMap instance;
    public InterpolatingTreeMap hoodTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap lowerWheelTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap tofTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap shuttlingRPSTreeMap = new InterpolatingTreeMap();
    public double kDrag = 0.001;

    public ShooterTreeMap() {

        // 2 MOTORS
//        hoodTreeMap.put(5.24, 2.5);
//        hoodTreeMap.put(4.04, 2.4);
//        hoodTreeMap.put(3.07, 2.0);
//        hoodTreeMap.put(2.47, 1.7);
//        hoodTreeMap.put(1.29, 0.8);
//
//        lowerWheelTreeMap.put(5.24, 58.0); // 47.41
//        lowerWheelTreeMap.put(4.04, 52.0); //43.13
//        lowerWheelTreeMap.put(3.07, 47.0); // 41.05
//        lowerWheelTreeMap.put(2.47, 44.0); // 39.71
//        lowerWheelTreeMap.put(1.29, 41.0); // 39.67
//
//        tofTreeMap.put(4.97, 1.32); // 1.32 1.33
//        tofTreeMap.put(3.89, 1.20); //1.19 1.21 1.22 1.15
//        tofTreeMap.put(2.78, 1.13); //0.71 0.96 0.66 1.13 1.13
//        tofTreeMap.put(2.18, 1.09); //1.05 1.06 1.12 1.10 1.09
//        tofTreeMap.put(1.39, 1.03); // 1.02 1.02 1.06 1.04


        // 1 MOTOR
        hoodTreeMap.put(5.24, 2.5);
        hoodTreeMap.put(4.04, 2.4);
        hoodTreeMap.put(3.07, 2.0);
        hoodTreeMap.put(2.47, 1.7);
        hoodTreeMap.put(1.29, 0.8);

        lowerWheelTreeMap.put(5.24, 60.0); // 47.41
        lowerWheelTreeMap.put(4.04, 54.0); //43.13
        lowerWheelTreeMap.put(3.07, 49.0); // 41.05
        lowerWheelTreeMap.put(2.47, 46.0); // 39.71
        lowerWheelTreeMap.put(1.29, 43.0); // 39.67

        tofTreeMap.put(4.97, 1.32); // 1.32 1.33
        tofTreeMap.put(3.89, 1.20); //1.19 1.21 1.22 1.15
        tofTreeMap.put(2.78, 1.13); //0.71 0.96 0.66 1.13 1.13
        tofTreeMap.put(2.18, 1.09); //1.05 1.06 1.12 1.10 1.09
        tofTreeMap.put(1.39, 1.03); // 1.02 1.02 1.06 1.04


        shuttlingRPSTreeMap.put(1.0, 20.0);
        shuttlingRPSTreeMap.put(3.0, 40.0);
        shuttlingRPSTreeMap.put(4.999, 45.0);
        shuttlingRPSTreeMap.put(5.0, 50.0);
        shuttlingRPSTreeMap.put(7.0, 65.0);
        shuttlingRPSTreeMap.put(9.0, 70.0);
        shuttlingRPSTreeMap.put(16.0, 75.0);

        updateDashboardMap();
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }


    public static ShooterTreeMap getInstance() {
        if (instance == null) {
            instance = new ShooterTreeMap();
        }
        return instance;
    }

    public double getRealTof(double dist) {
//        double tof = tofTreeMap.getInterpolated(dist);
//        return (1 - Math.exp(-kDrag * tof)) / kDrag;
        // return 1.15 + (0.04 * dist)
        return tofTreeMap.getInterpolated(dist);
    }

    public void addShot(double dist, double RPM) {
         instance.lowerWheelTreeMap.put(dist, RPM);
         updateDashboardMap();
    }

    public void removeShot(double dist) {
        if (instance.lowerWheelTreeMap.containsKey(dist)) {
            instance.lowerWheelTreeMap.remove(dist);
            updateDashboardMap();
        }
    }

    public double getMinDistance() {
        return 1.7;
    }

    public double getMaxDistance() {
        return 5.13;
    }

    private void updateDashboardMap() {
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }
}

