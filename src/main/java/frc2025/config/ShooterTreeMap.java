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
    public double kDrag = 0.001;
//    public InterpolatingTreeMap shuttlingHoodTreeMap = new InterpolatingTreeMap();
    
    public ShooterTreeMap() {
        hoodTreeMap.put(5.20, 2.7);
        hoodTreeMap.put(4.83, 2.3);
        hoodTreeMap.put(4.65, 2.25);
        hoodTreeMap.put(4.14, 2.2);
        hoodTreeMap.put(3.77, 1.7);
        hoodTreeMap.put(3.40, 1.5);
        hoodTreeMap.put(2.94, 1.25);
        hoodTreeMap.put(2.37, 1.2);
        hoodTreeMap.put(2.05, 1.0);
        hoodTreeMap.put(1.57, 0.9);
        hoodTreeMap.put(0.97, 0.5);

        lowerWheelTreeMap.put(5.20, 52.5);
        lowerWheelTreeMap.put(4.65, 52.0);
        lowerWheelTreeMap.put(4.14, 49.0);
        lowerWheelTreeMap.put(2.94, 49.0);
        lowerWheelTreeMap.put(2.37, 45.0);
        lowerWheelTreeMap.put(2.05, 45.0);
        lowerWheelTreeMap.put(1.57, 42.0);
        lowerWheelTreeMap.put(0.0, 42.0);

        tofTreeMap.put(4.32, 1.29);
        tofTreeMap.put(3.87, 1.26);
        tofTreeMap.put(3.09, 1.38);
        tofTreeMap.put(2.92, 1.39);
        tofTreeMap.put(2.63, 1.33);
        tofTreeMap.put(2.20, 1.30);
        tofTreeMap.put(1.47, 1.28);

//        hoodTreeMap.put(5.20, 2.7);
//        hoodTreeMap.put(4.83, 2.3);
//        hoodTreeMap.put(4.65, 2.25);
//        hoodTreeMap.put(4.14, 2.2);
//        hoodTreeMap.put(3.77, 1.7);
//        hoodTreeMap.put(3.40, 1.5);
//        hoodTreeMap.put(2.94, 1.25);
//        hoodTreeMap.put(2.37, 1.2);
//        hoodTreeMap.put(2.05, 1.0);
//        hoodTreeMap.put(1.57, 0.9);
//        hoodTreeMap.put(0.97, 0.5);
//
//        lowerWheelTreeMap.put(5.20, 52.5);
//        lowerWheelTreeMap.put(4.65, 52.0);
//        lowerWheelTreeMap.put(4.14, 49.0);
//        lowerWheelTreeMap.put(2.94, 49.0);
//        lowerWheelTreeMap.put(2.37, 45.0);
//        lowerWheelTreeMap.put(2.05, 45.0);
//        lowerWheelTreeMap.put(1.57, 42.0);
//        lowerWheelTreeMap.put(0.0, 42.0);
//
//        tofTreeMap.put(4.32, 1.29);
//        tofTreeMap.put(3.87, 1.26);
//        tofTreeMap.put(3.09, 1.38);
//        tofTreeMap.put(2.92, 1.39);
//        tofTreeMap.put(2.63, 1.33);
//        tofTreeMap.put(2.20, 1.30);
//        tofTreeMap.put(1.47, 1.28);
//
//        radialVelocityTreeMap.put(-0.10, 0.50);
//        radialVelocityTreeMap.put(-0.5, 1.5);
//        radialVelocityTreeMap.put(0.10, -0.50);
//        radialVelocityTreeMap.put(0.5, -1.5);

        radialVelocityTreeMap.put(-0.10, 0.00);
        radialVelocityTreeMap.put(-0.5, 0.00);
        radialVelocityTreeMap.put(0.10, 0.00);
        radialVelocityTreeMap.put(0.5, 0.00);

        shuttlingRPSTreeMap.put(3.0, 45.0);
        shuttlingRPSTreeMap.put(4.999, 50.0);
        shuttlingRPSTreeMap.put(5.0, 55.0);
        shuttlingRPSTreeMap.put(7.0, 60.0);
        shuttlingRPSTreeMap.put(9.0, 75.0);
        shuttlingRPSTreeMap.put(16.0, 80.0);

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

    public double getMaxDistance() {
        return 6.0;
    }

    private void updateDashboardMap() {
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }
}

