package frc2025.config;

import frc2025.util.InterpolatingTreeMap;

public class ShooterTreeMap {
    private static ShooterTreeMap instance;
    public InterpolatingTreeMap pivotTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap upperWheelTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap lowerWheelTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap tofTreeMap = new InterpolatingTreeMap();

    public ShooterTreeMap() {
        pivotTreeMap.put(0.0, 15.0);
        pivotTreeMap.put(10.0, 5.0);

        lowerWheelTreeMap.put(2.5, 1750.0);
        lowerWheelTreeMap.put(3.0, 1900.0);
        lowerWheelTreeMap.put(4.0, 2300.0);
        lowerWheelTreeMap.put(4.2, 2500.0);
        lowerWheelTreeMap.put(5.0, 3000.0);

        tofTreeMap.put(0.0, 1.0);
        tofTreeMap.put(1.0, 1.0);
        tofTreeMap.put(2.0, 1.0);
    }

    public static ShooterTreeMap getInstance() {
        if (instance == null) {
            return new ShooterTreeMap();
        }
        return getInstance();
    }
}
