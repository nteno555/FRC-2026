package frc2025.config;

import java.util.ArrayList;

import frc2025.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTreeMap {
    private static ShooterTreeMap instance;
    public  InterpolatingTreeMap pivotTreeMap = new InterpolatingTreeMap();
    public  InterpolatingTreeMap upperWheelTreeMap = new InterpolatingTreeMap();
    public  InterpolatingTreeMap lowerWheelTreeMap = new InterpolatingTreeMap();
    public  InterpolatingTreeMap tofTreeMap = new InterpolatingTreeMap();
    
    public ShooterTreeMap() {
        pivotTreeMap.put(0.0, 15.0);
        pivotTreeMap.put(10.0, 5.0);

        lowerWheelTreeMap.put(1.5, -45.4);
        lowerWheelTreeMap.put(2.1, -51.2);
        lowerWheelTreeMap.put(3.0, -60.8);
        lowerWheelTreeMap.put(4.0, -72.0);
        lowerWheelTreeMap.put(5.0, -88.0);
//        lowerWheelTreeMap.put()

        tofTreeMap.put(0.0, 0.5);
        tofTreeMap.put(1.0, 1.0);
        tofTreeMap.put(2.0, 1.2);
        tofTreeMap.put(3.0, 1.3);
        tofTreeMap.put(4.0, 1.37);
        tofTreeMap.put(5.0, 1.42);
        tofTreeMap.put(6.0, 1.55);
        tofTreeMap.put(10.0, 2.0);
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

    private void updateDashboardMap() {
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }
}

