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
        tofTreeMap.put(5.13, 1.32); // 1.33 1.29 1.30
        tofTreeMap.put(4.06, 1.28); // 1.33 1.28 1.32
        tofTreeMap.put(3.06, 1.16); // 1.17 1.1 1.18
        tofTreeMap.put(1.46, 1.09);


        lowerWheelTreeMap.put(5.05, 30.0); // 24.22
        lowerWheelTreeMap.put(4.10, 27.5 ); // 21.85
        lowerWheelTreeMap.put(3.03, 24.0); // 20.78
        lowerWheelTreeMap.put(2.12, 22.0); // 19.95
        lowerWheelTreeMap.put(1.57, 20.5); // 19.10

        hoodTreeMap.put(5.05, 2.5);
        hoodTreeMap.put(4.10, 2.2);
        hoodTreeMap.put(3.03, 1.95);
        hoodTreeMap.put(2.12, 1.6);
        hoodTreeMap.put(1.57, 1.3);
        // vy = rps * sin(85 - 12.28*hood)

        shuttlingRPSTreeMap.put(3.0, 20.0);
        shuttlingRPSTreeMap.put(6.0, 30.0);
        shuttlingRPSTreeMap.put(10.0, 40.0);
        shuttlingRPSTreeMap.put(20.0, 70.0);

        shuttlingTofTreeMap.put(1.0, 0.5);
        shuttlingTofTreeMap.put(16.0, 4.0);

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

        lowerWheelTreeMap.put(5.24, 58.0); // 47.41
        lowerWheelTreeMap.put(4.04, 50.0); // 43.13
        lowerWheelTreeMap.put(3.07, 45.0); // 41.05
        lowerWheelTreeMap.put(2.47, 43.0); // 39.71
        lowerWheelTreeMap.put(1.29, 40.0); // 39.67
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
