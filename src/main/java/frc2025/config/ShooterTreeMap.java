package frc2025.config;

import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc2025.subsystems.hood.HoodSubsystem;
import frc2025.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

        lowerWheelTreeMap.put(1.4367, 18.2);
        lowerWheelTreeMap.put(1.928, 20.5);
        lowerWheelTreeMap.put(2.3132, 22.9);
        lowerWheelTreeMap.put(2.7972, 24.8);
        lowerWheelTreeMap.put(3.2983, 28.0);
        lowerWheelTreeMap.put(3.6102, 30.4);
        lowerWheelTreeMap.put(4.3841, 36.8);
        
        // default RPS is 27.0
        // hoodTreeMap.put(1.5029, null);
        // hoodTreeMap.put(1.5029, null);
        // hoodTreeMap.put(1.5029, null);
        // hoodTreeMap.put(1.5029, null);
        // hoodTreeMap.put(1.5029, null);
        // hoodTreeMap.put(1.5029, null);
        
        // default RPS 32.0
        // hoodTreeMap.put(4.3871, 1.1);
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

