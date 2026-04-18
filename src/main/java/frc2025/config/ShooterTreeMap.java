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
        // tofTreeMap
        tofTreeMap.put(1.29, 0.743);
        tofTreeMap.put(2.00, 0.777);
        tofTreeMap.put(2.47, 0.813);
        tofTreeMap.put(3.07, 0.858);
        tofTreeMap.put(4.04, 0.927);
        tofTreeMap.put(4.50, 0.976);
        tofTreeMap.put(5.24, 1.071);
        tofTreeMap.put(6.00, 1.162);
        // lowerWheelTreeMap
        // lowerWheelTreeMap.put(1.29, 18.2 * 1.15);
        // lowerWheelTreeMap.put(2.00, 19.9 * 1.12);
        // lowerWheelTreeMap.put(2.47, 21.2 * 1.10);
        // lowerWheelTreeMap.put(3.07, 22.8 * 1.10);
        // lowerWheelTreeMap.put(4.04, 25.5 * 1.10);
        // lowerWheelTreeMap.put(4.50, 26.8 * 1.10);
        // lowerWheelTreeMap.put(5.24, 28.7 * 1.07);
        // lowerWheelTreeMap.put(6.00, 30.7 * 1.07);
        lowerWheelTreeMap.put(1.318, 21.6); // 20.5717493
        lowerWheelTreeMap.put(2.00, 23.0); // 20.737
        lowerWheelTreeMap.put(2.55, 24.5); // 21.9733822
        lowerWheelTreeMap.put(3.252, 25.5); // 22.0836478
        lowerWheelTreeMap.put(4.006, 27.75); // 22.9037381
        lowerWheelTreeMap.put(4.353, 28.7); // 23.1449601
        lowerWheelTreeMap.put(5.212, 30.5); // 23.3643555
        lowerWheelTreeMap.put(6.00, 30.7);
        // hoodTreeMap
        hoodTreeMap.put(1.318, 1.02);
        hoodTreeMap.put(2.00, 1.65);
        hoodTreeMap.put(2.55, 1.70);
        hoodTreeMap.put(3.252, 2.000);
        hoodTreeMap.put(4.006, 2.35);
        hoodTreeMap.put(4.353, 2.5);
        hoodTreeMap.put(5.212, 2.800);
        hoodTreeMap.put(6.00, 2.800);

        // rps * sin(85 - 12.5*hood) 

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
