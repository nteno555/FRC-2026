package frc2025.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX indexer = new TalonFX(SubsystemConstants.indexerID, CANBus.roboRIO());
    private final TalonFX indexer2 = new TalonFX(SubsystemConstants.indexerID2, CANBus.roboRIO());
    private final boolean NEED_DOUBLE_INDEXER = SubsystemConstants.needDoubleIndexer;

    private final DutyCycleOut speedRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public IndexerIOReal() {
        indexer.getConfigurator().apply(new TalonFXConfiguration());
        if (NEED_DOUBLE_INDEXER) indexer2.getConfigurator().apply(new TalonFXConfiguration());

        var indexerConfigs = new TalonFXConfiguration();
        
        indexerConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.indexerSupplyCurrentLimit;
        indexerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        indexerConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.indexerStatorCurrentLimit;
        indexerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = 100;
        configTorque.PeakReverseTorqueCurrent = 100;
        configTorque.TorqueNeutralDeadband = SubsystemConstants.indexerTorqueNeutralDeadband;

        indexerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        indexer.getConfigurator().apply(indexerConfigs, 0.050);
        indexer.getConfigurator().apply(configTorque, 0.05);
        indexer2.getConfigurator().apply(indexerConfigs, 0.050);
        indexer2.getConfigurator().apply(configTorque, 0.05);

        indexer2.setControl(new Follower(indexer.getDeviceID(), MotorAlignmentValue.Opposed));


    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVoltage = indexer.getMotorVoltage().getValueAsDouble();
        inputs.indexer2Voltage = indexer2.getMotorVoltage().getValueAsDouble();
        inputs.indexer2Current = indexer2.getStatorCurrent().getValueAsDouble();
        inputs.indexerCurrent = indexer.getStatorCurrent().getValueAsDouble();
        inputs.connected = indexer.isConnected();

        inputs.totalCurrent = indexer.getStatorCurrent().getValueAsDouble() + indexer2.getStatorCurrent().getValueAsDouble();
        if (NEED_DOUBLE_INDEXER) inputs.totalCurrent += indexer2.getStatorCurrent().getValueAsDouble();


        Logger.recordOutput("Current/Indexer", indexer.getStatorCurrent().getValueAsDouble());
        if (NEED_DOUBLE_INDEXER) Logger.recordOutput("Current/Indexer", indexer2.getStatorCurrent().getValueAsDouble());

        
    }

    @Override
    public void setSpeed(double indexerSpeed) {
        indexer.setControl(speedRequest.withOutput(indexerSpeed));
    }

    @Override
    public void setVoltage(double indexerVoltage) {
        indexer.setControl(voltageRequest.withOutput(indexerVoltage));
    }


}