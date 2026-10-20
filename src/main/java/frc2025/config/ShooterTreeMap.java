package frc2025.config;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2025.util.InterpolatingTreeMap;

public class ShooterTreeMap {
    public static InterpolatingTreeMap hoodTreeMap = new InterpolatingTreeMap();
    public static InterpolatingTreeMap lowerWheelTreeMap = new InterpolatingTreeMap();
    public static InterpolatingTreeMap tofTreeMap = new InterpolatingTreeMap();
    public static InterpolatingTreeMap radialVelocityTreeMap = new InterpolatingTreeMap();
    public static InterpolatingTreeMap shuttlingTofTreeMap = new InterpolatingTreeMap();
    public static InterpolatingTreeMap shuttlingRPSTreeMap = new InterpolatingTreeMap();
    public static InterpolatingTreeMap shuttlingHoodTreeMap = new InterpolatingTreeMap();

    static {
        updateDashboardMap();
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());

        
        lowerWheelTreeMap.put(1.4367, 18.2); // 1
        lowerWheelTreeMap.put(1.9647,22.8); // 2
        lowerWheelTreeMap.put(2.4111, 23.5); // 3
        lowerWheelTreeMap.put(2.8889, 23.8); // 4
        lowerWheelTreeMap.put(3.2965, 25.6); // 5
        lowerWheelTreeMap.put(3.8140, 27.1); // 6
        lowerWheelTreeMap.put(4.3200, 30.0); // 7
        // lowerWheelTreeMap.put(4.4185, 28.2); // 8
        // default RPS is 27.0
        hoodTreeMap.put(1.4367, 0.0); // 1
        hoodTreeMap.put(1.9647, 0.0); // 2
        hoodTreeMap.put(2.4111, 0.0); // 3
        hoodTreeMap.put(2.8889, 0.1); // 4
        hoodTreeMap.put(3.2965, 0.1); // 5
        hoodTreeMap.put(3.8140, 0.2); // 6
        hoodTreeMap.put(4.3200, 0.3); // 7

        
        /*
         * Dist: 1.4367, rps: 18.2, hood: 0.0
        Dist: 1.9647, rps: 22.8, hood: 0.0
        Dist: 2.4111, rps: 24.0, hood: 0.1
        Dist: 2.8889, rps: 24.8, hood: 0.2
        Dist: 3.2965, rps: 26.0, hood: 0.2
        Dist: 3.814, rps: 28.1, hood: 0.3
        Dist: 4.3200, rps: 30.3 hood: 0.4

         */

        //TODO: tof tree map for shuttling and shooting, rps and hood tree map for shuttling
        shuttlingRPSTreeMap.put(2.0,38.0);
        shuttlingRPSTreeMap.put(6.0,48.0);
        shuttlingRPSTreeMap.put(10.0,57.0);
        shuttlingRPSTreeMap.put(16.0,65.0);
        shuttlingRPSTreeMap.put(100.0,70.0);

        shuttlingHoodTreeMap.put(2.0,2.0);
        shuttlingHoodTreeMap.put(6.0,2.3);
        shuttlingHoodTreeMap.put(10.0,2.5);
        shuttlingHoodTreeMap.put(16.0,3.0);
        shuttlingHoodTreeMap.put(100.0,3.3);
    }


    public static void addShot(double dist, double RPM) {
         lowerWheelTreeMap.put(dist, RPM);
         updateDashboardMap();
    }

    public static void removeShot(double dist) {
        if (lowerWheelTreeMap.containsKey(dist)) {
            lowerWheelTreeMap.remove(dist);
            updateDashboardMap();
        }
    }

    private static void updateDashboardMap() {
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }
}

