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
        hoodTreeMap.put(5.24, 2.5);
        hoodTreeMap.put(4.04, 2.4);
        hoodTreeMap.put(3.07, 2.0);
        hoodTreeMap.put(2.47, 1.7);
        hoodTreeMap.put(1.29, 0.8);

        lowerWheelTreeMap.put(5.24, 58.0); // 47.41
        lowerWheelTreeMap.put(4.04, 52.0); //43.13
        lowerWheelTreeMap.put(3.07, 47.0); // 41.05
        lowerWheelTreeMap.put(2.47, 44.0); // 39.71
        lowerWheelTreeMap.put(1.29, 41.0); // 39.67

        tofTreeMap.put(4.97, 1.32); // 1.32 1.33
        tofTreeMap.put(3.89, 1.20); //1.19 1.21 1.22 1.15
        tofTreeMap.put(2.78, 1.13); //0.71 0.96 0.66 1.13 1.13
        tofTreeMap.put(2.18, 1.09); //1.05 1.06 1.12 1.10 1.09
        tofTreeMap.put(1.39, 1.03); // 1.02 1.02 1.06 1.04
//        hoodTreeMap.put(5.14, 2.5);
////        hoodTreeMap.put(5.04, 2.55);
//        hoodTreeMap.put(4.47, 2.4);
//        hoodTreeMap.put(3.93, 2.2);
//        hoodTreeMap.put(3.354, 1.9);
//        hoodTreeMap.put(2.584, 1.5);
//        hoodTreeMap.put(2.09, 1.1);
//        hoodTreeMap.put(1.55, 1.0);
//        hoodTreeMap.put(1.11, 1.0);
//
//        // 59 * sin(2.5/2.7 * (50-85) + 85 deg) = 46.9 (5.13 m)
//        // 54 * sin(2.4/2.7 * (50-85) + 85 deg) = 43.6 (4.47 m)
//        // 53 * sin(2.2/2.7 * (50-85) + 85 deg) = 44.2 (3.93 m)
//        // 51 * sin(1.9/2.7 * (50-85) + 85 deg) = 44.33 (3.354 m)
//        // 49 * sin(1.5/2.7 * (50-85) + 85 deg) = 44.6 (2.584 m)
//        // 49 * sin(1.1/2.7 * (50-85) + 85 deg) = 46.3 (2.09 m)
//        // 46 * sin(1.0/2.7 * (50-85) + 85 deg) = 43.75 (1.559 m)
//        // 42 * sin(1.0/2.7 * (50-85) + 85 deg) = 40.0 (1.11 m)
//
//        lowerWheelTreeMap.put(5.13, 59.0); //
////        lowerWheelTreeMap.put(5.04, 57.0);
//        lowerWheelTreeMap.put(4.47, 54.0);
//        lowerWheelTreeMap.put(3.93, 53.0);
//        lowerWheelTreeMap.put(3.354, 51.0);
//        lowerWheelTreeMap.put(2.584, 49.0);
//        lowerWheelTreeMap.put(2.09, 49.0);
//        lowerWheelTreeMap.put(1.559, 46.0);
//        lowerWheelTreeMap.put(1.11, 42.0);

//        lowerWheelTreeMap.put(5.04, 55.0);
//        lowerWheelTreeMap.put(4.47, 51.0);
//        lowerWheelTreeMap.put(3.93, 51.0);
//        lowerWheelTreeMap.put(3.354, 49.0);
//        lowerWheelTreeMap.put(2.584, 46.0);
//        lowerWheelTreeMap.put(2.09, 46.0);
//        lowerWheelTreeMap.put(1.559, 43.0);
//        lowerWheelTreeMap.put(1.11, 39.0);
//        tofTreeMap.put(1.559, 1.22);
//        tofTreeMap.put(2.09, 1.21);
//        tofTreeMap.put(2.584, 1.27);
//        tofTreeMap.put(3.20, 1.26);
////        tofTreeMap.put(3.96, 1.24);
//        tofTreeMap.put(4.38, 1.33);
//        tofTreeMap.put(5.08, 1.35);


//        tofTreeMap.put();


//        hoodTreeMap.put(5.12, 2.7);
//        hoodTreeMap.put(4.23, 2.65);
//        hoodTreeMap.put(3.50, 2.05);
//        hoodTreeMap.put(2.51, 1.45);
//        hoodTreeMap.put(1.67, 1.1);
//        hoodTreeMap.put(1.12, 0.9);
//
//        lowerWheelTreeMap.put(5.12, 58.0);
//        lowerWheelTreeMap.put(4.23, 50.0);
//        lowerWheelTreeMap.put(3.50, 50.0);
//        lowerWheelTreeMap.put(2.51, 47.0);
//        lowerWheelTreeMap.put(1.67, 44.0);
//        lowerWheelTreeMap.put(1.12, 37.5);
//
//        tofTreeMap.put(5.12, 1.26);
//        tofTreeMap.put(4.23, 1.16);
//        tofTreeMap.put(2.51, 1.13);
//        tofTreeMap.put(1.67, 1.12);

//        hoodTreeMap.put(5.07, 2.7);
//        hoodTreeMap.put(4.38, 2.6);
//        hoodTreeMap.put(3.94, 2.45);
//        hoodTreeMap.put(3.42, 2.4);
//        hoodTreeMap.put(2.91, 2.3);
//        hoodTreeMap.put(2.53, 2.2);
//        hoodTreeMap.put(2.10, 2.0);
//        hoodTreeMap.put(1.79, 1.95);
//        hoodTreeMap.put(1.52, 1.7);
//        hoodTreeMap.put(1.12, 0.9);
//
//        lowerWheelTreeMap.put(5.07, 59.0);
//        lowerWheelTreeMap.put(4.38, 54.5);
//        lowerWheelTreeMap.put(3.94, 51.0);
//        lowerWheelTreeMap.put(3.42, 48.0);
//        lowerWheelTreeMap.put(2.91, 46.0);
//        lowerWheelTreeMap.put(2.53, 43.5);
//        lowerWheelTreeMap.put(2.10, 40.0);
//        lowerWheelTreeMap.put(1.79, 39.0);
//        lowerWheelTreeMap.put(1.52, 38.5);
//        lowerWheelTreeMap.put(1.12, 37.5);
//
//        // redo these
//        tofTreeMap.put(5.12, 1.13);
//        tofTreeMap.put(4.25, 1.16);
//        tofTreeMap.put(3.52, 1.19);
//        tofTreeMap.put(2.64, 1.12);
//        tofTreeMap.put(1.58, 0.98);
//
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

