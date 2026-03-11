package frc2025.util;

import edu.wpi.first.wpilibj.DriverStation;

import java.sql.Driver;

public class Alliance {
    public static boolean get() {
        if(!DriverStation.getAlliance().isPresent()) return true;
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
}
