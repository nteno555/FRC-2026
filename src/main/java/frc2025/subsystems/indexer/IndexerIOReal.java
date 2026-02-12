package frc2025.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerIOReal implements IndexerIOIF {
    private final TalonFX indexer = new TalonFX(0,"rio");


    public void IndexerIoReal() {
        indexer.getConfigurator().apply(new TalonFXConfiguration());
        var indexerConfigs = new TalonFXConfiguration();
        indexerConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        indexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfigs.CurrentLimits.StatorCurrentLimit = 40;
        indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = 40;
        configTorque.PeakReverseTorqueCurrent = 40;
        configTorque.TorqueNeutralDeadband = 1;
        indexer.getConfigurator().apply(indexerConfigs);
    }

    private ControlRequest createControlRequest(double targetValue, boolean useVoltage) {
        if (useVoltage) {
            return new VoltageOut(targetValue);
        } else {
            return new DutyCycleOut(targetValue);
        }
    }
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVoltage = indexer.getMotorVoltage().getValueAsDouble();
        inputs.indexerCurrent = indexer.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setSpeed(double indexerSpeed) {
        indexer.setControl(createControlRequest(indexerSpeed, false));
    }

    @Override
    public void setVoltage(double indexerVoltage) {
        indexer.setControl(createControlRequest(indexerVoltage, true));
    }

}
