package frc2025.subsystems.shooter;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;

public class ShooterIOReal implements ShooterIO {
    private TalonFX flywheelMaster = new TalonFX(SubsystemConstants.flywheelMasterID, "rio");
    private TalonFX flywheelSlaveA1 = new TalonFX(SubsystemConstants.flywheelSlaveA1ID, "rio");
    private TalonFX flywheelSlaveB1 = new TalonFX(SubsystemConstants.flywheelSlaveB1ID, "rio");
    private TalonFX flywheelSlaveB2 = new TalonFX(SubsystemConstants.flywheelSlaveB2ID, "rio");

    private TalonFX tower = new TalonFX(SubsystemConstants.towerID, "rio");

    public ShooterIOReal() {
        flywheelMaster.getConfigurator().apply(new TalonFXConfiguration());
        flywheelSlaveA1.getConfigurator().apply(new TalonFXConfiguration());
        flywheelSlaveB1.getConfigurator().apply(new TalonFXConfiguration());
        flywheelSlaveB2.getConfigurator().apply(new TalonFXConfiguration());
        tower.getConfigurator().apply(new TalonFXConfiguration());

        var flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kS = SubsystemConstants.flywheelkS;
        flywheelConfig.Slot0.kV = SubsystemConstants.flywheelkV;
        flywheelConfig.Slot0.kA = SubsystemConstants.flywheelkA;
        flywheelConfig.Slot0.kP = SubsystemConstants.flywheelkP;
        flywheelConfig.Slot0.kI = SubsystemConstants.flywheelkI;
        flywheelConfig.Slot0.kD = SubsystemConstants.flywheelkD;

        flywheelConfig.Slot1.kS = SubsystemConstants.flywheelkSAmps;
        flywheelConfig.Slot1.kV = SubsystemConstants.flywheelkVAmps;
        flywheelConfig.Slot1.kA = SubsystemConstants.flywheelkAAmps;
        flywheelConfig.Slot1.kP = SubsystemConstants.flywheelkPAmps;
        flywheelConfig.Slot1.kI = SubsystemConstants.flywheelkIAmps;
        flywheelConfig.Slot1.kD = SubsystemConstants.flywheelkDAmps;

        flywheelConfig.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.flywheelSupplyCurrentLimit;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = SubsystemConstants.flywheelStatorCurrentLimit;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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


        towerConfigs.CurrentLimits.SupplyCurrentLimit = 200;
        towerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        towerConfigs.CurrentLimits.StatorCurrentLimit = 200;
        towerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs towerConfigTorque =  new TorqueCurrentConfigs();
        towerConfigTorque.PeakForwardTorqueCurrent = 200;
        towerConfigTorque.PeakReverseTorqueCurrent = 200;
        towerConfigTorque.TorqueNeutralDeadband = 1;

        flywheelMaster.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelSlaveA1.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelSlaveB1.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelSlaveB2.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelMaster.setNeutralMode(NeutralModeValue.Coast);
        flywheelSlaveA1.setNeutralMode(NeutralModeValue.Coast);
        flywheelSlaveB1.setNeutralMode(NeutralModeValue.Coast);
        flywheelSlaveB2.setNeutralMode(NeutralModeValue.Coast);

        flywheelSlaveA1.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Aligned));
        flywheelSlaveB1.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Opposed));
        flywheelSlaveB2.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Opposed));

        tower.getConfigurator().apply(towerConfigs, 0.050);
        tower.getConfigurator().apply(towerConfigTorque, 0.05);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.masterVoltage = flywheelMaster.getMotorVoltage().getValueAsDouble();
        inputs.masterVelocityRPS = flywheelMaster.getVelocity().getValueAsDouble();

        inputs.slaveA1Voltage = flywheelSlaveA1.getMotorVoltage().getValueAsDouble();
        inputs.slaveA1VelocityRPS = flywheelSlaveA1.getVelocity().getValueAsDouble();
        inputs.slaveB1Voltage = flywheelSlaveB1.getMotorVoltage().getValueAsDouble();
        inputs.slaveB1VelocityRPS = flywheelSlaveB1.getVelocity().getValueAsDouble();
        inputs.slaveB2Voltage = flywheelSlaveB2.getMotorVoltage().getValueAsDouble();
        inputs.slaveB2VelocityRPS = flywheelSlaveB2.getVelocity().getValueAsDouble();

        inputs.towerVoltage = tower.getMotorVoltage().getValueAsDouble();
        inputs.towerRPS = tower.getVelocity().getValueAsDouble();
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
//        flywheelMaster.setControl(new VelocityVoltage(speed)
//                .withEnableFOC(true));

        flywheelMaster.setControl(
                new VelocityTorqueCurrentFOC(speed)
                        .withSlot(1)
        );
        tower.setControl(new VelocityVoltage(speed * SubsystemConstants.towerFlywheelRatio)
                .withEnableFOC(true));

//        tower.setControl(
//                new VelocityTorqueCurrentFOC(speed * SubsystemConstants.towerFlywheelRatio)
//                        .withSlot(1)
//        );
    }
}
