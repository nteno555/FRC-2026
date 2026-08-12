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
    private TalonFX flywheelAligned = new TalonFX(SubsystemConstants.flywheelAlignedID, CANBus.roboRIO());
    private TalonFX flywheelOpposedA = new TalonFX(SubsystemConstants.flywheelOpposedAID, CANBus.roboRIO());
    private TalonFX flywheelOpposedB = new TalonFX(SubsystemConstants.flywheelOpposedBID, CANBus.roboRIO());
//    private TalonFX tower = new TalonFX(SubsystemConstants.towerID, CANBus.roboRIO());

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityTorqueCurrentFOC speedRequest = new VelocityTorqueCurrentFOC(0).withSlot(1);
    private final VelocityVoltage speedVoltageRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final VoltageOut bangbangVoltageRequest = new VoltageOut(12.0).withEnableFOC(true);
    private final VelocityVoltage towerSpeedVoltageRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final TorqueCurrentFOC currentRequest = new TorqueCurrentFOC(0);

    public ShooterIOReal() {
        flywheelMaster.getConfigurator().apply(new TalonFXConfiguration());
        flywheelAligned.getConfigurator().apply(new TalonFXConfiguration());
        flywheelOpposedA.getConfigurator().apply(new TalonFXConfiguration());
        flywheelOpposedB.getConfigurator().apply(new TalonFXConfiguration());

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
        flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;

        flywheelConfig.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.flywheelSupplyCurrentLimit;
        flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = SubsystemConstants.flywheelStatorCurrentLimit;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        var slaveConfig = new TalonFXConfiguration();
        slaveConfig.Slot0 = flywheelConfig.Slot0;
        slaveConfig.Slot1 = flywheelConfig.Slot1;
        slaveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        flywheelMaster.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelAligned.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelOpposedA.getConfigurator().apply(slaveConfig, 0.050);
        flywheelOpposedB.getConfigurator().apply(slaveConfig, 0.050);

        flywheelMaster.setNeutralMode(NeutralModeValue.Coast);
        flywheelAligned.setNeutralMode(NeutralModeValue.Coast);
        flywheelOpposedA.setNeutralMode(NeutralModeValue.Coast);
        flywheelOpposedB.setNeutralMode(NeutralModeValue.Coast);

        int masterID = flywheelMaster.getDeviceID();
        var alignedStatus = flywheelAligned.setControl(new Follower(masterID, MotorAlignmentValue.Aligned));
        var opposedAStatus = flywheelOpposedA.setControl(new Follower(masterID, MotorAlignmentValue.Opposed));
        var opposedBStatus = flywheelOpposedB.setControl(new Follower(masterID, MotorAlignmentValue.Opposed));

        Logger.recordOutput("Shooter/alignedFollowerStatus", alignedStatus.toString());
        Logger.recordOutput("Shooter/opposedAFollowerStatus", opposedAStatus.toString());
        Logger.recordOutput("Shooter/opposedBFollowerStatus", opposedBStatus.toString());

        // tower.getConfigurator().apply(towerConfigs, 0.050);
        // tower.getConfigurator().apply(towerConfigTorque, 0.05);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.masterVoltage = flywheelMaster.getMotorVoltage().getValueAsDouble();
        inputs.masterVelocityRPS = flywheelMaster.getVelocity().getValueAsDouble();

        inputs.alignedVoltage = flywheelAligned.getMotorVoltage().getValueAsDouble();
        inputs.alignedVelocityRPS = flywheelAligned.getVelocity().getValueAsDouble();

        inputs.opposedAVoltage = flywheelOpposedA.getMotorVoltage().getValueAsDouble();
        inputs.opposedAVelocityRPS = flywheelOpposedA.getVelocity().getValueAsDouble();

        inputs.opposedBVoltage = flywheelOpposedB.getMotorVoltage().getValueAsDouble();
        inputs.opposedBVelocityRPS = flywheelOpposedB.getVelocity().getValueAsDouble();

        inputs.totalCurrent = flywheelMaster.getStatorCurrent().getValueAsDouble()
                + flywheelAligned.getStatorCurrent().getValueAsDouble() + flywheelOpposedA.getStatorCurrent().getValueAsDouble()
                + flywheelOpposedB.getStatorCurrent().getValueAsDouble();

        Logger.recordOutput("Current/Shooter Master", flywheelMaster.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Current/Shooter Aligned Follower", flywheelAligned.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Current/Shooter Opposed Master", flywheelOpposedA.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Current/Shooter Opposed Follower", flywheelOpposedB.getStatorCurrent().getValueAsDouble());
    }

    private final Follower alignedFollower = new Follower(SubsystemConstants.flywheelMasterID, MotorAlignmentValue.Aligned);
    private final Follower opposedAFollower = new Follower(SubsystemConstants.flywheelMasterID, MotorAlignmentValue.Opposed);
    private final Follower opposedBFollower = new Follower(SubsystemConstants.flywheelMasterID, MotorAlignmentValue.Opposed);
    @Override
    public void setVoltage(double voltage) {
        flywheelMaster.setControl(voltageRequest.withOutput(voltage));
        flywheelAligned.setControl(alignedFollower);
        flywheelOpposedA.setControl(opposedAFollower);
        flywheelOpposedB.setControl(opposedBFollower);
        // flywheelAligned.setControl(new Follower(13, MotorAlignmentValue.Aligned));
        // flywheelOpposedA.setControl(new Follower(13, MotorAlignmentValue.Opposed));
        // flywheelOpposedB.setControl(new Follower(13, MotorAlignmentValue.Opposed));
    }

    @Override
    public void setCurrent(double currentAmps) {
        flywheelMaster.setControl(currentRequest.withOutput(currentAmps));
        flywheelAligned.setControl(alignedFollower);
        flywheelOpposedA.setControl(opposedAFollower);
        flywheelOpposedB.setControl(opposedBFollower);
        // flywheelAligned.setControl(currentRequest.withOutput(currentAmps));
        // flywheelOpposedA.setControl(currentRequest.withOutput(currentAmps));
        // flywheelOpposedB.setControl(currentRequest.withOutput(currentAmps));
        // flywheelAligned.setControl(new Follower(13, MotorAlignmentValue.Aligned));
        // flywheelOpposedA.setControl(new Follower(13, MotorAlignmentValue.Opposed));
        // flywheelOpposedB.setControl(new Follower(13, MotorAlignmentValue.Opposed));
    }

    // @Override
    // public void setSpeed(double speed, double acceleration) {
    //     double errorRPS = speed - flywheelMaster.getVelocity().getValueAsDouble();
    //     boolean wantBangBang = false;

    //     if (speed < 64.0) {
    //         Logger.recordOutput("Shooter/bangbang", wantBangBang);
    //         Logger.recordOutput("Shooter/usingVoltage", false);
    //         if (wantBangBang) {
    //             flywheelMaster.setControl(bangbangVoltageRequest);
    //             flywheelAligned.setControl(alignedFollower);
    //             flywheelOpposedA.setControl(opposedAFollower);
    //             flywheelOpposedB.setControl(opposedBFollower);
    //             // flywheelAligned.setControl(new Follower(13, MotorAlignmentValue.Aligned));
    //             // flywheelOpposedA.setControl(new Follower(13, MotorAlignmentValue.Opposed));
    //             // flywheelOpposedB.setControl(new Follower(13, MotorAlignmentValue.Opposed));
    //         } else {
    //             flywheelMaster.setControl(speedRequest.withVelocity(speed));
    //             flywheelAligned.setControl(alignedFollower);
    //             flywheelOpposedA.setControl(opposedAFollower);
    //             flywheelOpposedB.setControl(opposedBFollower);
    //             // flywheelAligned.setControl(new Follower(13, MotorAlignmentValue.Aligned));
    //             // flywheelOpposedA.setControl(new Follower(13, MotorAlignmentValue.Opposed));
    //             // flywheelOpposedB.setControl(new Follower(13, MotorAlignmentValue.Opposed));
    //         }
    //     }
    //     else {
    //         Logger.recordOutput("Shooter/usingVoltage", true);
    //         flywheelMaster.setControl(
    //                 speedVoltageRequest.withVelocity(speed));
    //         flywheelAligned.setControl(alignedFollower);
    //         flywheelOpposedA.setControl(opposedAFollower);
    //         flywheelOpposedB.setControl(opposedBFollower);
    //         // flywheelAligned.setControl(new Follower(13, MotorAlignmentValue.Aligned));
    //         // flywheelOpposedA.setControl(new Follower(13, MotorAlignmentValue.Opposed));
    //         // flywheelOpposedB.setControl(new Follower(13, MotorAlignmentValue.Opposed));
    //     }
    // }
    @Override
    public void setSpeed(double speed, double acceleration) {
        flywheelMaster.setControl(
                   speedVoltageRequest.withVelocity(speed));
    }
}