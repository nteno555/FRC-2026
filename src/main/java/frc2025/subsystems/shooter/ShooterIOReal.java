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

    private VelocityTorqueCurrentFOC torqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(1);
    private VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

    public ShooterIOReal() {
        flywheelMaster.getConfigurator().apply(new TalonFXConfiguration());
        flywheelSlaveA1.getConfigurator().apply(new TalonFXConfiguration());
        flywheelSlaveB1.getConfigurator().apply(new TalonFXConfiguration());
        flywheelSlaveB2.getConfigurator().apply(new TalonFXConfiguration());

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
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        //voltage+velocity
        inputs.masterVoltage = flywheelMaster.getMotorVoltage().getValueAsDouble();
        inputs.masterVelocityRPS = flywheelMaster.getVelocity().getValueAsDouble();

        inputs.slaveA1Voltage = flywheelSlaveA1.getMotorVoltage().getValueAsDouble();
        inputs.slaveA1VelocityRPS = flywheelSlaveA1.getVelocity().getValueAsDouble();
        inputs.slaveB1Voltage = flywheelSlaveB1.getMotorVoltage().getValueAsDouble();
        inputs.slaveB1VelocityRPS = flywheelSlaveB1.getVelocity().getValueAsDouble();
        inputs.slaveB2Voltage = flywheelSlaveB2.getMotorVoltage().getValueAsDouble();
        inputs.slaveB2VelocityRPS = flywheelSlaveB2.getVelocity().getValueAsDouble();

        inputs.masterStatorCurrent = flywheelMaster.getStatorCurrent().getValueAsDouble();
        inputs.slaveA1StatorCurrent = flywheelSlaveA1.getStatorCurrent().getValueAsDouble();
        inputs.slaveB1StatorCurrent = flywheelSlaveB1.getStatorCurrent().getValueAsDouble();
        inputs.slaveB2StatorCurrent = flywheelSlaveB2.getStatorCurrent().getValueAsDouble();
        inputs.totalCurrent = inputs.masterStatorCurrent+inputs.slaveA1StatorCurrent+inputs.slaveB1StatorCurrent+inputs.slaveB2StatorCurrent;

    }

    @Override
    public void setVoltage(double voltage) {
        flywheelMaster.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        flywheelMaster.setControl(new DutyCycleOut(percentOutput));
    }

    @Override
    public void setSpeed(double speed) {
        flywheelMaster.setControl(velocityVoltageRequest.withVelocity(speed));
    
    }

}
