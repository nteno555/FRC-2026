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
    public InterpolatingTreeMap radialVelocityTreeaMap = new InterpolatingTreeMap();
    
    public ShooterTreeMap() {
//        hoodTreeMap.put(0.9, 0.5);
////        hoodTreeMap.put(1.15, 0.7);
//        hoodTreeMap.put(1.32, 0.8);
//        hoodTreeMap.put(1.51, 0.9);
//        hoodTreeMap.put(1.63, 1.0);
////        hoodTreeMap.put(1.84, 1.1);
////        hoodTreeMap.put(2.22, 1.2);
//        hoodTreeMap.put(2.36, 1.3);
//        hoodTreeMap.put(3.22, 1.6);
//        hoodTreeMap.put(3.93, 1.8);
//        hoodTreeMap.put(4.13, 1.9);
////        hoodTreeMap.put(4.38, 2.0);
////        hoodTreeMap.put(4.85, 2.1);
//        hoodTreeMap.put(5.2, 2.2);

//        lowerWheelTreeMap.put(0.9, 35.0);
////        lowerWheelTreeMap.put(1.15, 37.0);
//        lowerWheelTreeMap.put(1.32, 37.5);
//        lowerWheelTreeMap.put(1.51, 38.0);
//        lowerWheelTreeMap.put(1.63, 38.5);
////        lowerWheelTreeMap.put(1.84, 39.0);
////        lowerWheelTreeMap.put(2.22, 40.0);
//        lowerWheelTreeMap.put(2.36, 40.5);
//        lowerWheelTreeMap.put(3.22, 42.5);
////        lowerWheelTreeMap.put(3.93, 44.5);
//        lowerWheelTreeMap.put(4.13, 45.0);
////        lowerWheelTreeMap.put(4.38, 46.0);
////        lowerWheelTreeMap.put(4.85, 46.5);
//        lowerWheelTreeMap.put(5.2, 48.0);

        hoodTreeMap.put(2.04, 1.4);
        hoodTreeMap.put(2.27, 1.4);
        hoodTreeMap.put(3.07, 1.4);
        hoodTreeMap.put(3.75, 1.4);
        hoodTreeMap.put(3.751, 2.0);
        hoodTreeMap.put(4.11, 2.0);
        hoodTreeMap.put(5.14, 2.0);

        lowerWheelTreeMap.put(2.04, 36.0);
        lowerWheelTreeMap.put(2.27, 38.0);
        lowerWheelTreeMap.put(2.39, 39.0);
        lowerWheelTreeMap.put(2.67, 40.0);
        lowerWheelTreeMap.put(3.07, 41.0);
        lowerWheelTreeMap.put(3.75, 45.0);
        lowerWheelTreeMap.put(3.751, 43.0);
        lowerWheelTreeMap.put(3.89, 43.5);
        lowerWheelTreeMap.put(4.11, 44.0);
        lowerWheelTreeMap.put(5.14, 51.0);

        tofTreeMap.put(1.0, 1.03);
        tofTreeMap.put(2.39, 1.17);
        tofTreeMap.put(2.67, 1.20);
        tofTreeMap.put(3.89, 1.26);

        radialVelocityTreeaMap.put(-0.10, 1.0);
        radialVelocityTreeaMap.put(-0.40, 2.0);
        radialVelocityTreeaMap.put(0.10, 1.0);
        radialVelocityTreeaMap.put(0.4, 2.0);

        updateDashboardMap();
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());

    }

    public double calculateRps(double vel, double dist) {
        double rps = -0.9 * (
                9.92021947
                        + 13.69360457 * dist
                        - 4.20965329  * vel
                        - 4.28770715 * dist * dist
                        + 1.31239698 * dist * vel
                        + 0.25042203 * vel * vel
                        + 0.58348240 * dist * dist * dist
                        - 0.29685441 * dist * dist * vel
                        - 0.00509588 * dist * vel * vel
                        - 0.00455643 * vel * vel * vel);

        if (rps < -40) rps = -40;
        return rps;
    }

    public double calculateHoodAngle(double vel, double dist) {
        return
                1.21404126
                        - 0.03121454 * dist
                        + 0.02514251 * vel
                        - 0.00335640 * dist * dist
                        + 0.01371330 * dist * vel
                        - 0.00787229 * vel * vel
                        - 0.00191794 * dist * dist * vel
                        + 0.00268512 * dist * vel * vel
                        - 0.00085436 * vel * vel * vel;
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
        return 5.20;
    }

    private void updateDashboardMap() {
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }
}

