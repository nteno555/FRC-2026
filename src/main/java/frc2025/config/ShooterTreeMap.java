package frc2025.config;

import java.util.ArrayList;

import frc2025.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTreeMap {
    private static ShooterTreeMap instance;
    public  InterpolatingTreeMap pivotTreeMap = new InterpolatingTreeMap();
    public  InterpolatingTreeMap lowerWheelTreeMap = new InterpolatingTreeMap();
    public  InterpolatingTreeMap tofTreeMap = new InterpolatingTreeMap();
    
    public ShooterTreeMap() {
//        pivotTreeMap.put(1.2, 0.0);
//        pivotTreeMap.put(1.5, 0.0);
//        pivotTreeMap.put(2.0, 0.0);
//        pivotTreeMap.put(2.66, 1.0);
//        pivotTreeMap.put(3.0, 1.5);
//        pivotTreeMap.put(4.9, -33.0);
        pivotTreeMap.put(2.0, 0.0);
        pivotTreeMap.put(2.25, 0.0);
        pivotTreeMap.put(2.50, 0.71);
        pivotTreeMap.put(2.75, 1.36);
        pivotTreeMap.put(3.0, 2.18);
        pivotTreeMap.put(3.25, 3.43);
        pivotTreeMap.put(3.5, 4.85);
//
        lowerWheelTreeMap.put(2.0, -24.0);
        lowerWheelTreeMap.put(2.25, -26.0);
        lowerWheelTreeMap.put(2.5, -26.0);
        lowerWheelTreeMap.put(2.75, -26.0);
        lowerWheelTreeMap.put(5.5, -31.0);

//        lowerWheelTreeMap.put(1.2, -22.0);
//        lowerWheelTreeMap.put(1.5, -23.0);
//        lowerWheelTreeMap.put(2.0, -25.0);
//        lowerWheelTreeMap.put(2.66, -26.0);
//        lowerWheelTreeMap.put(3.0, -28.0);
//        lowerWheelTreeMap.put(4.9, -33.0);

        //tofTreeMap.put(0.0, 0.0);

        tofTreeMap.put(2.0, 1.0);
//        tofTreeMap.put(0.5, 0.7);
//        tofTreeMap.put(1.0, 1.0);
//        tofTreeMap.put(3.0, 1.0);
//        tofTreeMap.put(2.0, 1.2);
//        tofTreeMap.put(3.0, 1.25);
//        tofTreeMap.put(4.0, 1.28);
//        tofTreeMap.put(5.0, 1.30);
//        tofTreeMap.put(6.0, 1.31);
        tofTreeMap.put(10.0, 1.0);
//        double tofAt2 = 0.6;
//        for(double k = 0; k <= 10; k += 0.1) {
//            tofTreeMap.put(k, tofAt2 * Math.pow(k / 2.0, 0.5) + 0.2);
//        }

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

