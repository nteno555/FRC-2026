package frc2025.config;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2025.util.InterpolatingTreeMap;

public class ShooterTreeMap {
    private static ShooterTreeMap instance;
    public InterpolatingTreeMap hoodTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap lowerWheelTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap tofTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap radialVelocityTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap shuttlingTofTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap shuttlingRPSTreeMap = new InterpolatingTreeMap();


    
    public ShooterTreeMap() {


        updateDashboardMap();
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());

        lowerWheelTreeMap.put(1.4367, 18.2); // 1
        lowerWheelTreeMap.put(1.9647,22.8); // 2
        lowerWheelTreeMap.put(2.4111, 24.0); // 3
        lowerWheelTreeMap.put(2.8889, 24.8); // 4
        lowerWheelTreeMap.put(3.2965, 26.0); // 5
        lowerWheelTreeMap.put(3.8140, 27.9); // 6
        lowerWheelTreeMap.put(4.3200, 30.0); // 7
        // lowerWheelTreeMap.put(4.4185, 28.2); // 8
        
        // default RPS is 27.0
        hoodTreeMap.put(1.4367, 0.0); // 1
        hoodTreeMap.put(1.9647, 0.0); // 2
        hoodTreeMap.put(2.4111, 0.1); // 3
        hoodTreeMap.put(2.8889, 0.2); // 4
        hoodTreeMap.put(3.2965, 0.2); // 5
        hoodTreeMap.put(3.8140, 0.3); // 6
        hoodTreeMap.put(4.3200, 0.4); // 7

        /*
         * Dist: 1.4367, rps: 18.2, hood: 0.0
        Dist: 1.9647, rps: 22.8, hood: 0.0
        Dist: 2.4111, rps: 24.0, hood: 0.1
        Dist: 2.8889, rps: 24.8, hood: 0.2
        Dist: 3.2965, rps: 26.0, hood: 0.2
        Dist: 3.814, rps: 28.1, hood: 0.3
        Dist: 4.3200, rps: 30.3 hood: 0.4

         */
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

    private void updateDashboardMap() {
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }
}

