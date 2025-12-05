package frc2025.config;

import frc2025.util.InterpolatingTreeMap;

public class ShooterTreeMap {
    private static ShooterTreeMap instance;
    public InterpolatingTreeMap pivotTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap upperWheelTreeMap = new InterpolatingTreeMap();
    public InterpolatingTreeMap lowerWheelTreeMap = new InterpolatingTreeMap();

    public ShooterTreeMap() {
        pivotTreeMap.put(0.0, 15.0);
        pivotTreeMap.put(10.0, 5.0);

        upperWheelTreeMap.put(0.0, 50.0);
        lowerWheelTreeMap.put(0.0, 50.0);
    }

    public static ShooterTreeMap getInstance() {
        if (instance == null) {
            return new ShooterTreeMap();
        }
        return getInstance();
    }
}
