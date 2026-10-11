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
    public InterpolatingTreeMap shuttlingTofTreeMap = new InterpolatingTreeMap();

    public double kDrag = 0.001;

    public ShooterTreeMap() {
        // tofTreeMap.put(5.13, 1.32); // 1.33 1.29 1.30
        // tofTreeMap.put(4.06, 1.28); // 1.33 1.28 1.32
        // tofTreeMap.put(3.06, 1.16); // 1.17 1.1 1.18
        // tofTreeMap.put(1.46, 1.09);

        // tofTreeMap.put(5.13, 1.64); // 1.33 1.29 1.30
        // tofTreeMap.put(4.025, 1.55); // 1.33 1.28 1.32
        // tofTreeMap.put(3.009, 1.36); // 1.17 1.1 1.18
        // tofTreeMap.put(1.485, 1.26); 

        // hood rotations: 1.17, 2.21, 3.05, 3.44, 4.08, 4.58

        // RPS: 22.6, 27.0, 32.0, 35.0, 38.4, 40.3
        
        // lowerWheelTreeMap.put(6.0, 58.0);
        // lowerWheelTreeMap.put(5.05, 52.0); // 24.22
        // lowerWheelTreeMap.put(4.10, 47.0); // 21.85
        // lowerWheelTreeMap.put(3.03, 42.0); // 20.78
        // lowerWheelTreeMap.put(2.12, 38.0); // 19.95
        // lowerWheelTreeMap.put(1.57, 36.0); // 19.10

        // hoodTreeMap.put(6.0, 2.55);
        // hoodTreeMap.put(5.05, 2.5);
        // hoodTreeMap.put(4.10, 2.2);
        // hoodTreeMap.put(3.03, 1.95);
        // hoodTreeMap.put(2.12, 1.6);
        // hoodTreeMap.put(1.57, 1.3);
        // lowerWheelTreeMap.put(1.543, 22.6);
        // lowerWheelTreeMap.put(2.178, 27.0); // 24.22
        // lowerWheelTreeMap.put(3.099, 32.0); // 21.85
        // lowerWheelTreeMap.put(4.152, 35.0); // 20.78
        // lowerWheelTreeMap.put(5.046, 38.4); // 19.95

        // lowerWheelTreeMap.put(6.0, 40.3); // 19.10
        /* 1.534   76.8d      1.82   26.4 -3.1/+3.1   1.06  70.7d
            2.220   73.3d      2.61   28.2 -2.5/+2.5   1.14  67.4d
            3.111   67.6d      3.86   29.6 -1.9/+1.9   1.16  60.9d
            3.939   62.2d      5.06   31.0 -1.5/+1.5   1.16  54.7d
            4.664   58.1d      5.98   32.4 -1.4/+1.4   1.16  50.3d
            5.060   56.3d      6.38   33.2 -1.2/+1.2   1.17  48.8d
            5.616   54.6d      6.76   34.8 -1.2/+1.2   1.20  47.8d */
        
        lowerWheelTreeMap.put(1.534, 26.4);
        lowerWheelTreeMap.put(2.220, 28.2);
        lowerWheelTreeMap.put(3.111, 29.6);
        lowerWheelTreeMap.put(3.939, 31.0);
        lowerWheelTreeMap.put(4.664, 32.4);
        lowerWheelTreeMap.put(5.060, 33.2);
        lowerWheelTreeMap.put(5.616, 34.8);

        hoodTreeMap.put(1.534, 1.82);
        hoodTreeMap.put(2.220, 2.61);
        hoodTreeMap.put(3.111, 3.86);
        hoodTreeMap.put(3.939, 5.06);
        hoodTreeMap.put(4.664, 5.98);
        hoodTreeMap.put(5.060, 6.38);
        hoodTreeMap.put(5.616, 6.76);

        tofTreeMap.put(1.534, 1.06);
        tofTreeMap.put(2.220, 1.14);
        tofTreeMap.put(3.111, 1.16);
        tofTreeMap.put(3.939, 1.16);
        tofTreeMap.put(4.664, 1.16);
        tofTreeMap.put(5.060, 1.17);
        tofTreeMap.put(5.616, 1.20);
        

        // TUNED

        // lowerWheelTreeMap.put(1.534, 27.0); // 15.8757785
        // lowerWheelTreeMap.put(2.220, 30.0); // 16.973333
        // lowerWheelTreeMap.put(3.111, 35.5); // 17.0999029
        // lowerWheelTreeMap.put(3.939, 40.0); // 18.9597874

        // lowerWheelTreeMap.put(4.664, 39.7); // 19.6070128

        // lowerWheelTreeMap.put(5.06, 39.7); // 19.6070128
        // lowerWheelTreeMap.put(5.06, 39.7); // 19.6070128
        // // lowerWheelTreeMap.put(5.616, 37.5); // 17.082934

        // hoodTreeMap.put(1.534, 1.5);
        // hoodTreeMap.put(2.220, 2.0);
        // hoodTreeMap.put(3.103, 2.5);
        // hoodTreeMap.put(3.939, 3.00);

        // hoodTreeMap.put(4.664, 3.8);

        // lowerWheelTreeMap.put(5.06, 39.7); // 19.6070128
        // lowerWheelTreeMap.put(5.06, 39.7); // 19.6070128
        // hoodTreeMap.put(5.616, 4.58);
        // vy = rps * sin(85 - 12.28*hood)
        //vy = rps · sin(0.09805 · hood_rotation)

        shuttlingRPSTreeMap.put(3.0, 35.0);
        shuttlingRPSTreeMap.put(6.0, 50.0);
        shuttlingRPSTreeMap.put(10.0, 70.0);
        shuttlingRPSTreeMap.put(20.0, 120.0);

        shuttlingTofTreeMap.put(1.0, 0.5);
        // shuttlingTofTreeMap.put(8.0, 2.0);
        // shuttlingTofTreeMap.put(8.01, 0.0);
        shuttlingTofTreeMap.put(16.0, 3.0);

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
        // double tof = tofTreeMap.getInterpolated(dist);
        // return (1 - Math.exp(-kDrag * tof)) / kDrag;
        // return 1.15 + (0.04 * dist)
        return tofTreeMap.getInterpolated(dist);
    }

    public double getRealTofShuttling(double dist) {
        return shuttlingTofTreeMap.getInterpolated(dist);
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

    public void changeRPS(double offset) {
        for (var key : lowerWheelTreeMap.keySet()) {
            lowerWheelTreeMap.replace(key, lowerWheelTreeMap.get(key) + offset);
        }
    }

    public void resetRPS() {
        lowerWheelTreeMap.clear();

        lowerWheelTreeMap.put(1.534, 26.4);
        lowerWheelTreeMap.put(2.220, 28.2);
        lowerWheelTreeMap.put(3.111, 29.6);
        lowerWheelTreeMap.put(3.939, 31.0);
        lowerWheelTreeMap.put(4.664, 32.4);
        lowerWheelTreeMap.put(5.060, 33.2);
        lowerWheelTreeMap.put(5.616, 34.8);
    }

    public double getMinDistance() {
        return 1.3;
    }

    public double getMaxDistance() {
        return 6.0;
    }

    private void updateDashboardMap() {
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }
}
