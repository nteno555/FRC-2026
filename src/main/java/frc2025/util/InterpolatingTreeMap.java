package frc2025.util;

import java.util.TreeMap;

public class InterpolatingTreeMap extends TreeMap<Double, Double> {
    public double interpolate(double lower, double upper, double norm) {
        return ((upper-lower)*norm) + lower;
    }

    public double inverseInterpolate(double lower, double upper, double interpolated) {
        if (upper == lower) return 0.0;
        return (interpolated-lower)/(upper-lower);
    }

    public Double getInterpolated(double key) {
        Double retrievedValue = get(key);
        if (retrievedValue == null) {
            // Get surrounding keys for interpolation
            Double upperKeyBound = ceilingKey(key);
            Double lowerKeyBound = floorKey(key);
            // If attempting interpolation at ends of tree, return the nearest data point
            if (upperKeyBound == null && lowerKeyBound == null) {
                return null;
            } else if (upperKeyBound == null) {
                return get(lowerKeyBound);
            } else if (lowerKeyBound == null) {
                return get(upperKeyBound);
            }
            // get surrounding values for interpolation
            Double upperValue = get(upperKeyBound);
            Double lowerValue = get(lowerKeyBound);
            return interpolate(lowerValue, upperValue, inverseInterpolate(lowerKeyBound, upperKeyBound, key));
        } else {
            return retrievedValue;
        }
    }
}