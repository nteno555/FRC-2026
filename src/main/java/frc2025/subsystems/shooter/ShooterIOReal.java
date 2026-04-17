package frc2025.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {
    private TalonFX flywheelMaster = new TalonFX(SubsystemConstants.flywheelMasterID, CANBus.roboRIO());
    private TalonFX flywheelSlave = new TalonFX(SubsystemConstants.flywheelSlaveID, CANBus.roboRIO());
    // private TalonFX tower = new TalonFX(SubsystemConstants.towerID,
    // CANBus.roboRIO());

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityTorqueCurrentFOC speedRequest = new VelocityTorqueCurrentFOC(0).withSlot(1);

    public ShooterIOReal() {
        flywheelMaster.getConfigurator().apply(new TalonFXConfiguration());
        // tower.getConfigurator().apply(new TalonFXConfiguration());

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

        flywheelConfig.Slot2.kS = SubsystemConstants.flywheelkSAmpsBangBang;
        flywheelConfig.Slot2.kV = SubsystemConstants.flywheelkVAmpsBangBang;
        flywheelConfig.Slot2.kA = SubsystemConstants.flywheelkAAmpsBangBang;
        flywheelConfig.Slot2.kP = SubsystemConstants.flywheelkPAmpsBangBang;
        flywheelConfig.Slot2.kI = SubsystemConstants.flywheelkIAmpsBangBang;
        flywheelConfig.Slot2.kD = SubsystemConstants.flywheelkDAmpsBangBang;
        flywheelConfig.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 200;
        flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -200; // uh this is really scuffed... for bangbang
                                                                      // control..

        flywheelConfig.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.flywheelSupplyCurrentLimit;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = SubsystemConstants.flywheelStatorCurrentLimit;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // var towerConfigs = new TalonFXConfiguration();
        //
        // towerConfigs.Slot0.kS = SubsystemConstants.towerkS;
        // towerConfigs.Slot0.kV = SubsystemConstants.towerkV;
        // towerConfigs.Slot0.kA = SubsystemConstants.towerkA;
        // towerConfigs.Slot0.kP = SubsystemConstants.towerkP;
        // towerConfigs.Slot0.kI = SubsystemConstants.towerkI;
        // towerConfigs.Slot0.kD = SubsystemConstants.towerkD;
        //
        // towerConfigs.Slot1.kS = SubsystemConstants.towerkSAmps;
        // towerConfigs.Slot1.kV = SubsystemConstants.towerkVAmps;
        // towerConfigs.Slot1.kA = SubsystemConstants.towerkAAmps;
        // towerConfigs.Slot1.kP = SubsystemConstants.towerkPAmps;
        // towerConfigs.Slot1.kI = SubsystemConstants.towerkIAmps;
        // towerConfigs.Slot1.kD = SubsystemConstants.towerkDAmps;
        //
        // towerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        //
        // towerConfigs.CurrentLimits.SupplyCurrentLimit = 200;
        // towerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        // towerConfigs.CurrentLimits.StatorCurrentLimit = 200;
        // towerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        //
        // TorqueCurrentConfigs towerConfigTorque = new TorqueCurrentConfigs();
        // towerConfigTorque.PeakForwardTorqueCurrent = 200;
        // towerConfigTorque.PeakReverseTorqueCurrent = 200;
        // towerConfigTorque.TorqueNeutralDeadband = 1;

        flywheelMaster.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelSlave.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelMaster.setNeutralMode(NeutralModeValue.Coast);
        flywheelSlave.setNeutralMode(NeutralModeValue.Coast);
        flywheelSlave.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Opposed));

        // tower.getConfigurator().apply(towerConfigs, 0.050);
        // tower.getConfigurator().apply(towerConfigTorque, 0.05);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.masterVoltage = flywheelMaster.getMotorVoltage().getValueAsDouble();
        inputs.masterVelocityRPS = flywheelMaster.getVelocity().getValueAsDouble();

        inputs.slaveVoltage = flywheelSlave.getMotorVoltage().getValueAsDouble();
        inputs.slaveVelocityRPS = flywheelSlave.getVelocity().getValueAsDouble();

        // inputs.towerVoltage = tower.getMotorVoltage().getValueAsDouble();
        // inputs.towerRPS = tower.getVelocity().getValueAsDouble();

        inputs.totalCurrent = flywheelMaster.getStatorCurrent().getValueAsDouble()
                + flywheelSlave.getStatorCurrent().getValueAsDouble();

        Logger.recordOutput("Current/Shooter Master", flywheelMaster.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Current/Shooter Slave", flywheelSlave.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void setVoltage(double voltage) {
        flywheelMaster.setControl(voltageRequest.withOutput(voltage));
        // tower.setControl(new VoltageOut(0.0));
    }

    @Override
    public void setSpeed(double speed, double acceleration) {
        double currentFF = speed < flywheelMaster.getVelocity().getValueAsDouble() ? -SubsystemConstants.flywheelkSAmps
                : SubsystemConstants.flywheelkSAmps;
        Logger.recordOutput("Shooter/currentFF", currentFF);

        if (speed < 1000.0) {
            Logger.recordOutput("Shooter/usingVoltage", false);
            flywheelMaster.setControl(
                    speedRequest.withVelocity(speed).withAcceleration(acceleration));
        } else {
            Logger.recordOutput("Shooter/usingVoltage", true);
            flywheelMaster.setControl(
                    voltageRequest.withOutput(speed * 0.127));
        }
        // tower.setControl(new VelocityVoltage(speed *
        // SubsystemConstants.towerFlywheelRatio)
        // .withEnableFOC(true));
    }
}
