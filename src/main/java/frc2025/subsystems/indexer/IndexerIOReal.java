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

    private final TalonFX tower = new TalonFX(SubsystemConstants.towerID, CANBus.roboRIO());
    private final TalonFX tower2 = new TalonFX(SubsystemConstants.towerID, CANBus.roboRIO());
    private final boolean HAS_TOWER = SubsystemConstants.hasTower;
    private final boolean NEED_DOUBLE_TOWER = SubsystemConstants.needDoubleTower;

    private final DutyCycleOut speedRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage towerSpeedRequest = new VelocityVoltage(0).withEnableFOC(true);

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
        if (NEED_DOUBLE_INDEXER) indexer2.getConfigurator().apply(indexerConfigs, 0.050);
        if (NEED_DOUBLE_INDEXER) indexer2.getConfigurator().apply(configTorque, 0.05);  

        if (NEED_DOUBLE_INDEXER) indexer2.setControl(new Follower(indexer.getDeviceID(), MotorAlignmentValue.Opposed));

        if (HAS_TOWER) {
            var towerConfigs = new TalonFXConfiguration();

            towerConfigs.Slot0.kS = SubsystemConstants.towerkS;
            towerConfigs.Slot0.kV = SubsystemConstants.towerkV;
            towerConfigs.Slot0.kA = SubsystemConstants.towerkA;
            towerConfigs.Slot0.kP = SubsystemConstants.towerkP;
            towerConfigs.Slot0.kI = SubsystemConstants.towerkI;
            towerConfigs.Slot0.kD = SubsystemConstants.towerkD;

            towerConfigs.Slot1.kS = SubsystemConstants.towerkSAmps;
            towerConfigs.Slot1.kV = SubsystemConstants.towerkVAmps;
            towerConfigs.Slot1.kA = SubsystemConstants.towerkAAmps;
            towerConfigs.Slot1.kP = SubsystemConstants.towerkPAmps;
            towerConfigs.Slot1.kI = SubsystemConstants.towerkIAmps;
            towerConfigs.Slot1.kD = SubsystemConstants.towerkDAmps;

            towerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            towerConfigs.CurrentLimits.SupplyCurrentLimit = 100;
            towerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
            towerConfigs.CurrentLimits.StatorCurrentLimit = 100;
            towerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

            TorqueCurrentConfigs towerConfigTorque =  new TorqueCurrentConfigs();
            towerConfigTorque.PeakForwardTorqueCurrent = 100;
            towerConfigTorque.PeakReverseTorqueCurrent = 100;
            towerConfigTorque.TorqueNeutralDeadband = 1;

            tower.getConfigurator().apply(towerConfigs, 0.050);
            tower.getConfigurator().apply(towerConfigTorque, 0.05);
            if (NEED_DOUBLE_TOWER) {
                tower2.getConfigurator().apply(towerConfigs, 0.050);
                tower2.getConfigurator().apply(towerConfigTorque, 0.05);
                tower2.setControl(new Follower(tower.getDeviceID(), MotorAlignmentValue.Opposed));
            }
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVoltage = indexer.getMotorVoltage().getValueAsDouble();
        inputs.indexerCurrent = indexer.getStatorCurrent().getValueAsDouble();
        inputs.connected = indexer.isConnected();

        if (NEED_DOUBLE_INDEXER) {
            inputs.indexerVoltage += indexer2.getMotorVoltage().getValueAsDouble();
            inputs.indexerCurrent += indexer2.getStatorCurrent().getValueAsDouble();
            inputs.connected = inputs.connected && indexer2.isConnected();
        }

        inputs.totalCurrent = indexer.getStatorCurrent().getValueAsDouble();
        if (NEED_DOUBLE_INDEXER) inputs.totalCurrent += indexer2.getStatorCurrent().getValueAsDouble();
        if (HAS_TOWER) {
            inputs.totalCurrent += tower.getStatorCurrent().getValueAsDouble();
            if (NEED_DOUBLE_TOWER) inputs.totalCurrent += tower2.getStatorCurrent().getValueAsDouble();
        }

        Logger.recordOutput("Current/Indexer", indexer.getStatorCurrent().getValueAsDouble());
        if (NEED_DOUBLE_INDEXER) Logger.recordOutput("Current/Indexer", indexer2.getStatorCurrent().getValueAsDouble());
        if (HAS_TOWER) {
            Logger.recordOutput("Current/Tower", tower.getStatorCurrent().getValueAsDouble());
            if (NEED_DOUBLE_TOWER) Logger.recordOutput("Current/Tower", tower.getStatorCurrent().getValueAsDouble());
        }
        
    }

    @Override
    public void setSpeed(double indexerSpeed) {
        indexer.setControl(speedRequest.withOutput(indexerSpeed));
        if (NEED_DOUBLE_INDEXER) indexer2.setControl(speedRequest.withOutput(indexerSpeed));
    }

    @Override
    public void setVoltage(double indexerVoltage) {
        indexer.setControl(voltageRequest.withOutput(indexerVoltage));
        if (NEED_DOUBLE_INDEXER) indexer2.setControl(voltageRequest.withOutput(indexerVoltage));
    }

    @Override
    public void setTowerSpeed(double speed) {
        if (HAS_TOWER) {
            tower.setControl(towerSpeedRequest.withVelocity(speed * SubsystemConstants.towerFlywheelRatio));
            if (NEED_DOUBLE_TOWER) tower2.setControl(towerSpeedRequest.withVelocity(speed * SubsystemConstants.towerFlywheelRatio));
        }
    }
}