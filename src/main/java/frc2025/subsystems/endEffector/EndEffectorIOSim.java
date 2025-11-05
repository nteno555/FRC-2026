package frc2025.subsystems.endEffector;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import org.littletonrobotics.junction.Logger;

public class EndEffectorIOSim implements EndEffectorIO{
    private double voltage;

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.current = 0;
        if (voltage < 0) {
            inputs.canRangeDistance = 999999;
        }
        if(voltage > 0) {
            inputs.canRangeDistance = 0;
            inputs.current = 9999;
        }
        inputs.voltage = voltage;
    }

    @Override
    public void setSpeed(double speed) {
        voltage = speed;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }
}
