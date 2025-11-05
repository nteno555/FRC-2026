package frc2025.subsystems.funnel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import org.littletonrobotics.junction.Logger;

public class FunnelIOReal implements FunnelIO{
    private final TalonFX funnelMaster = new TalonFX(14, "rio");

    public FunnelIOReal() {
        funnelMaster.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();
        
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 40;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = 40;
        configTorque.PeakReverseTorqueCurrent = 40;
        configTorque.TorqueNeutralDeadband = 1;


        funnelMaster.getConfigurator().apply(talonFXConfigs, 0.050);
        funnelMaster.getConfigurator().apply(configTorque, 0.05);

    }


    private ControlRequest createControlRequest(double targetValue, boolean useVoltage) {
        if (useVoltage) {
            return new VoltageOut(targetValue);
        } else {
            return new DutyCycleOut(targetValue);
        }
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs) {
        // TODO: not sure if this is needed
        inputs.voltage = funnelMaster.getMotorVoltage().getValueAsDouble();
        inputs.current = funnelMaster.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setSpeed(double speed) {
        funnelMaster.setControl(createControlRequest(speed, false));
    }

    @Override
    public void setVoltage(double voltage) {
        funnelMaster.setControl(createControlRequest(voltage, true));
    }
}
