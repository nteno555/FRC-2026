package frc2025.subsystems.shooter;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;

public class ShooterIOReal implements ShooterIO {
    private TalonFX flywheelMaster = new TalonFX(SubsystemConstants.flywheelMasterID, "rio");
    private TalonFX flywheelSlave = new TalonFX(SubsystemConstants.flywheelSlaveID, "rio");
    private TalonFX tower = new TalonFX(SubsystemConstants.towerID, "rio");

    public ShooterIOReal() {
		flywheelMaster.getConfigurator().apply(new TalonFXConfiguration());
        tower.getConfigurator().apply(new TalonFXConfiguration());

        var flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kS = SubsystemConstants.flywheelkS;
        flywheelConfig.Slot0.kV = SubsystemConstants.flywheelkV;
        flywheelConfig.Slot0.kA = SubsystemConstants.flywheelkA;
        flywheelConfig.Slot0.kP = SubsystemConstants.flywheelkP;
        flywheelConfig.Slot0.kI = SubsystemConstants.flywheelkI;
        flywheelConfig.Slot0.kD = SubsystemConstants.flywheelkD;

		flywheelConfig.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.flywheelSupplyCurrentLimit;
		flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.StatorCurrentLimit = SubsystemConstants.flywheelStatorCurrentLimit;
		flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        var towerConfigs = new TalonFXConfiguration();

        towerConfigs.CurrentLimits.SupplyCurrentLimit = 80;
        towerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        towerConfigs.CurrentLimits.StatorCurrentLimit = 80;
        towerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs towerConfigTorque =  new TorqueCurrentConfigs();
        towerConfigTorque.PeakForwardTorqueCurrent = 200;
        towerConfigTorque.PeakReverseTorqueCurrent = 200;
        towerConfigTorque.TorqueNeutralDeadband = 1;

        flywheelMaster.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelSlave.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelMaster.setNeutralMode(NeutralModeValue.Coast);
        flywheelSlave.setNeutralMode(NeutralModeValue.Coast);
        flywheelSlave.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Opposed));

        tower.getConfigurator().apply(towerConfigs, 0.050);
        tower.getConfigurator().apply(towerConfigTorque, 0.05);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.masterVoltage = flywheelMaster.getMotorVoltage().getValueAsDouble();
        inputs.masterVelocityRPM = flywheelMaster.getVelocity().getValueAsDouble();

        inputs.slaveVoltage = flywheelSlave.getMotorVoltage().getValueAsDouble();
        inputs.slaveVelocityRPM = flywheelSlave.getVelocity().getValueAsDouble();

        inputs.towerVoltage = tower.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        flywheelMaster.setControl(new VoltageOut(voltage));
        tower.setControl(new VoltageOut(0.0));
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        flywheelMaster.setControl(new DutyCycleOut(percentOutput));
    }

    @Override
    public void setSpeed(double speed) {
        flywheelMaster.setControl(new VelocityVoltage(speed)
                .withEnableFOC(true));
        tower.setControl(new VoltageOut(SubsystemConstants.towerShootVoltage));
    }
}
