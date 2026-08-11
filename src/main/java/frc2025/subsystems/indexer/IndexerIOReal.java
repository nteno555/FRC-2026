package frc2025.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX indexer = new TalonFX(SubsystemConstants.indexerID, CANBus.roboRIO());
    private final TalonFX tower = new TalonFX(SubsystemConstants.towerID, CANBus.roboRIO());

    private final DutyCycleOut speedRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage towerSpeedRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VoltageOut bangbangVoltageRequest = new VoltageOut(12.0).withEnableFOC(true);

    public IndexerIOReal() {
        indexer.getConfigurator().apply(new TalonFXConfiguration());

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
        towerConfigTorque.PeakReverseTorqueCurrent = 40;
        towerConfigTorque.TorqueNeutralDeadband = 1;

        tower.getConfigurator().apply(towerConfigs, 0.050);
        tower.getConfigurator().apply(towerConfigTorque, 0.05);

        tower.setControl(new Follower(SubsystemConstants.indexerID, MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVoltage = indexer.getMotorVoltage().getValueAsDouble();
        inputs.indexerCurrent = indexer.getStatorCurrent().getValueAsDouble();
        inputs.connected = indexer.isConnected();

        inputs.totalCurrent = indexer.getStatorCurrent().getValueAsDouble() + tower.getStatorCurrent().getValueAsDouble();

        Logger.recordOutput("Current/Indexer", indexer.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Current/Tower", tower.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void setSpeed(double indexerSpeed) {
        indexer.setControl(speedRequest.withOutput(indexerSpeed));
    }

    @Override
    public void setVoltage(double indexerVoltage) {
        indexer.setControl(voltageRequest.withOutput(indexerVoltage));
    }

    @Override
    public void setTowerSpeed(double speed) {
        speed = speed * SubsystemConstants.towerFlywheelRatio;
        boolean inTolerance = (speed - tower.getVelocity().getValueAsDouble()) < SubsystemConstants.towerBangBangThreshold;

        if (!inTolerance) {
            Logger.recordOutput("Tower/towerBangBang", true);
            tower.setControl(bangbangVoltageRequest);
        } else {
            Logger.recordOutput("Tower/towerBangBang", false);
            tower.setControl(towerSpeedRequest.withVelocity(speed));
        }

        Logger.recordOutput("Tower/setpoint", speed);
        Logger.recordOutput("Tower/towerRPS", tower.getVelocity().getValueAsDouble());
    }
}
