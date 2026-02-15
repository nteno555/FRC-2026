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
    public InterpolatingTreeMap radialVelocityTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap shuttlingRPSTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap shuttlingHoodTreeMap = new InterpolatingTreeMap();
    
    public ShooterTreeMap() {
        hoodTreeMap.put(0.5, 0.5);
        hoodTreeMap.put(1.50, 0.5);
        hoodTreeMap.put(1.501, 1.4);
        hoodTreeMap.put(2.04, 1.4);
        hoodTreeMap.put(3.75, 1.4);
        hoodTreeMap.put(3.751, 2.0);
        hoodTreeMap.put(6.0, 2.0);

        lowerWheelTreeMap.put(0.5, 38.0);
        lowerWheelTreeMap.put(1.50, 40.0);
        lowerWheelTreeMap.put(1.501, 35.0);
        lowerWheelTreeMap.put(2.04, 37.0);
        lowerWheelTreeMap.put(2.27, 39.0);
        lowerWheelTreeMap.put(2.39, 41.0);
        lowerWheelTreeMap.put(2.67, 42.0);
        lowerWheelTreeMap.put(3.07, 44.0);
        lowerWheelTreeMap.put(3.75, 47.0);
        lowerWheelTreeMap.put(3.751, 43.0);
        lowerWheelTreeMap.put(3.89, 43.5);
        lowerWheelTreeMap.put(4.11, 44.0);
        lowerWheelTreeMap.put(5.14, 51.0);
        lowerWheelTreeMap.put(6.0, 56.0);

        tofTreeMap.put(1.0, 1.03);
        tofTreeMap.put(2.39, 1.17);
        tofTreeMap.put(2.67, 1.20);
        tofTreeMap.put(3.513, 1.32);
        tofTreeMap.put(3.75, 1.35);
        tofTreeMap.put(3.751, 1.24);
        tofTreeMap.put(3.89, 1.26);
        tofTreeMap.put(5.22, 1.52);
        tofTreeMap.put(6.0, 1.60);

        radialVelocityTreeMap.put(-0.10, 0.50);
        radialVelocityTreeMap.put(-0.5, 1.5);
        radialVelocityTreeMap.put(0.10, -0.50);
        radialVelocityTreeMap.put(0.5, -1.5);

        shuttlingRPSTreeMap.put(3.0, 47.0);
        shuttlingRPSTreeMap.put(4.999, 52.0);
        shuttlingRPSTreeMap.put(5.0, 55.0);
        shuttlingRPSTreeMap.put(7.0, 60.0);
        shuttlingRPSTreeMap.put(9.0, 75.0);
        shuttlingRPSTreeMap.put(16.0, 80.0);

        shuttlingHoodTreeMap.put(3.0, 2.0);
        shuttlingHoodTreeMap.put(4.999, 2.0);
        shuttlingHoodTreeMap.put(5.0, 2.8);
        shuttlingHoodTreeMap.put(16.0, 2.8);

        updateDashboardMap();
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());

    }

    public static ShooterTreeMap getInstance() {
        if (instance == null) {
            instance = new ShooterTreeMap();
        }
        return instance;
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

    public double getMaxDistance() {
        return 6.0;
    }

    private void updateDashboardMap() {
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }
}

