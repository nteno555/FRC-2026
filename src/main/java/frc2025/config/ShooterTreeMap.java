package frc2025.config;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.subsystems.hood.HoodSubsystem;
import frc2025.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTreeMap {
    private static ShooterTreeMap instance;
    public  InterpolatingTreeMap hoodTreeMap = new InterpolatingTreeMap();
    public  InterpolatingTreeMap lowerWheelTreeMap = new InterpolatingTreeMap();
    public  InterpolatingTreeMap tofTreeMap = new InterpolatingTreeMap();
    
    public ShooterTreeMap() {
        hoodTreeMap.put(2.0, 0.0);
        hoodTreeMap.put(2.50, 1.5);
        hoodTreeMap.put(3.0, 3.0);
        hoodTreeMap.put(3.5, 4.5);
        hoodTreeMap.put(4.0, 6.0);

        lowerWheelTreeMap.put(2.0, -24.0);
        lowerWheelTreeMap.put(2.50, -26.0);
        lowerWheelTreeMap.put(3.0, -27.5);
        lowerWheelTreeMap.put(3.5, -28.5);
        lowerWheelTreeMap.put(4.0, -29.3);
        lowerWheelTreeMap.put(5.0, -31.0);

//        tofTreeMap.put(1.3, 1.15);
//        tofTreeMap.put(2.4, 1.27);
//        tofTreeMap.put(3.5, 1.41);
//        tofTreeMap.put(4.5, 1.50);
//        tofTreeMap.put(9.5, 1.90);
        tofTreeMap.put(2.0, 1.25);
//        tofTreeMap.put(2.28, 1.09);
        tofTreeMap.put(4.0, 1.25);
//        double tofAt2 = 0.6;
//        for(double k = 0; k <= 10; k += 0.1) {
//            tofTreeMap.put(k, tofAt2 * Math.pow(k / 2.0, 0.5) + 0.2);
//        }

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

    private void updateDashboardMap() {
        SmartDashboard.putString("Tuning/lowerWheelTree Map", lowerWheelTreeMap.toString());
    }
}

