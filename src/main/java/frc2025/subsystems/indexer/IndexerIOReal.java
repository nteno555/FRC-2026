package frc2025.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX indexer = new TalonFX(21, "rio");
    private final TalonFX tower = new TalonFX(25, "rio");

    public IndexerIOReal() {
        indexer.getConfigurator().apply(new TalonFXConfiguration());
        tower.getConfigurator().apply(new TalonFXConfiguration());

        var indexerConfigs = new TalonFXConfiguration();
        
        indexerConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        indexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfigs.CurrentLimits.StatorCurrentLimit = 40;
        indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = 40;
        configTorque.PeakReverseTorqueCurrent = 40;
        configTorque.TorqueNeutralDeadband = 1;


        var towerConfigs = new TalonFXConfiguration();

        towerConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        towerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        towerConfigs.CurrentLimits.StatorCurrentLimit = 40;
        towerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs towerConfigTorque =  new TorqueCurrentConfigs();
        towerConfigTorque.PeakForwardTorqueCurrent = 40;
        towerConfigTorque.PeakReverseTorqueCurrent = 40;
        towerConfigTorque.TorqueNeutralDeadband = 1;

        indexer.getConfigurator().apply(indexerConfigs, 0.050);
        indexer.getConfigurator().apply(configTorque, 0.05);
        tower.getConfigurator().apply(towerConfigs, 0.050);
        tower.getConfigurator().apply(towerConfigTorque, 0.05);
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
        inputs.towerVoltage = tower.getMotorVoltage().getValueAsDouble();
        inputs.towerCurrent = tower.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setSpeed(double indexerSpeed, double towerSpeed) {
        indexer.setControl(createControlRequest(indexerSpeed, false));
        tower.setControl(createControlRequest(towerSpeed, false));
    }

    @Override
    public void setVoltage(double indexerVoltage, double towerVoltage) {
        indexer.setControl(createControlRequest(indexerVoltage, true));
        tower.setControl(createControlRequest(towerVoltage, false));
    }


}
