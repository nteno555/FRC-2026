package frc2025.util;

import edu.wpi.first.wpilibj.DriverStation;

public class Alliance {
    public static boolean get() {
        if (!DriverStation.getAlliance().isPresent()) return false;
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}
