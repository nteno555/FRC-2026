package frc2025.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX indexer = new TalonFX(SubsystemConstants.indexerID, CANBus.roboRIO());

    public IndexerIOReal() {
        indexer.getConfigurator().apply(new TalonFXConfiguration());

        var indexerConfigs = new TalonFXConfiguration();
        
        indexerConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.indexerSupplyCurrentLimit;
        indexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.indexerStatorCurrentLimit;
        indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = SubsystemConstants.indexerPeakForwardTorqueCurrent;
        configTorque.PeakReverseTorqueCurrent = SubsystemConstants.indexerPeakReverseTorqueCurrent;
        configTorque.TorqueNeutralDeadband = SubsystemConstants.indexerTorqueNeutralDeadband;

        indexerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        indexer.getConfigurator().apply(indexerConfigs, 0.050);
        indexer.getConfigurator().apply(configTorque, 0.05);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVoltage = indexer.getMotorVoltage().getValueAsDouble();
        inputs.indexerCurrent = indexer.getTorqueCurrent().getValueAsDouble();
        inputs.connected = indexer.isConnected();
    }

    @Override
    public void setSpeed(double indexerSpeed) {
        indexer.setControl(new DutyCycleOut(indexerSpeed));
    }

    @Override
    public void setVoltage(double indexerVoltage) {
        indexer.setControl(new VoltageOut(indexerVoltage));
    }
}
