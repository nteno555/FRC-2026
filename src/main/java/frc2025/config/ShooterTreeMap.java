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
        hoodTreeMap.put(5.07, 2.7);
        hoodTreeMap.put(4.38, 2.6);
        hoodTreeMap.put(3.94, 2.45);
        hoodTreeMap.put(3.42, 2.4);
        hoodTreeMap.put(2.91, 2.3);
        hoodTreeMap.put(2.53, 2.2);
        hoodTreeMap.put(2.10, 2.0);
        hoodTreeMap.put(1.79, 1.95);
        hoodTreeMap.put(1.52, 1.7);
        hoodTreeMap.put(1.12, 0.9);

        lowerWheelTreeMap.put(5.07, 59.0);
        lowerWheelTreeMap.put(4.38, 54.5);
        lowerWheelTreeMap.put(3.94, 51.0);
        lowerWheelTreeMap.put(3.42, 48.0);
        lowerWheelTreeMap.put(2.91, 46.0);
        lowerWheelTreeMap.put(2.53, 43.5);
        lowerWheelTreeMap.put(2.10, 40.0);
        lowerWheelTreeMap.put(1.79, 39.0);
        lowerWheelTreeMap.put(1.52, 38.5);
        lowerWheelTreeMap.put(1.12, 37.5);

        // redo these
        tofTreeMap.put(5.07, 1.35);
        tofTreeMap.put(3.70, 1.19);
        tofTreeMap.put(3.50, 1.13);
        tofTreeMap.put(2.44, 1.10);
        tofTreeMap.put(1.12, 1.02);

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
        double tof = tofTreeMap.getInterpolated(dist);
        return (1 - Math.exp(-kDrag * tof)) / kDrag;
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
        return 5.07;
    }

    private void updateDashboardMap() {
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }
}

