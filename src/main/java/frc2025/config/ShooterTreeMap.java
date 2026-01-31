package frc2025.config;

import frc2025.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTreeMap {
    private static ShooterTreeMap instance;
    public  InterpolatingTreeMap hoodTreeMap = new InterpolatingTreeMap();
    public  InterpolatingTreeMap lowerWheelTreeMap = new InterpolatingTreeMap();
    public  InterpolatingTreeMap tofTreeMap = new InterpolatingTreeMap();
    
    public ShooterTreeMap() {
        hoodTreeMap.put(2.0, 0.0);
        hoodTreeMap.put(2.25, 0.0);
        hoodTreeMap.put(2.50, 0.0);
        hoodTreeMap.put(3.0, 0.0);
        hoodTreeMap.put(3.5, 0.0);
        hoodTreeMap.put(4.0, 0.0);
        hoodTreeMap.put(5.0, 2.0);
        lowerWheelTreeMap.put(2.0, -24.4);
        lowerWheelTreeMap.put(2.50, -27.4);
        lowerWheelTreeMap.put(3.0, -29.4);
        lowerWheelTreeMap.put(3.5, -31.4);
        lowerWheelTreeMap.put(4.0, -33.4);
        lowerWheelTreeMap.put(5.0, -33.4);

        tofTreeMap.put(1.3, 1.15);
        tofTreeMap.put(2.4, 1.27);
        tofTreeMap.put(3.5, 1.41);
        tofTreeMap.put(4.5, 1.50);
        tofTreeMap.put(9.5, 1.90);
//        double tofAt2 = 0.6;
//        for(double k = 0; k <= 10; k += 0.1) {
//            tofTreeMap.put(k, tofAt2 * Math.pow(k / 2.0, 0.5) + 0.2);
//        }

        updateDashboardMap();
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());

    }

    public double calculateRps(double vel, double dist) {
        return 12.54232786
                        + 8.081028    * dist
                        - 6.61004747  * vel
                        - 1.54118737  * dist * dist
                        + 1.932908    * dist * vel
                        + 1.95712032  * vel * vel
                        + 0.15451226  * dist * dist * dist
                        - 0.21478905  * dist * dist * vel
                        - 0.32202871  * dist * vel * vel
                        - 0.18796412  * vel * vel * vel;

        /*
        12.54232786
                        + 8.081028    * x
                        - 6.61004747  * y
                        - 1.54118737  * x * x
                        + 1.932908    * x * y
                        + 1.95712032  * y * y
                        + 0.15451226  * x * x * x
                        - 0.21478905  * x * x * y
                        - 0.32202871  * x * y * y
                        - 0.18796412  * y * y * y;
         */
    }

    public double calculateHoodAngle(double vel, double dist) {
        return 1.22173048
                        - 0.05235988 * dist
                        - 0.05235988 * vel;
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

