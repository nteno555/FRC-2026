package frc2025.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeDeployMaster = new TalonFX(SubsystemConstants.intakeDeployMasterID, "rio");
    private final TalonFX intakeDeploySlave = new TalonFX(SubsystemConstants.intakeDeploySlaveID, "rio");
    private final TalonFX intakeRoller = new TalonFX(SubsystemConstants.intakeRollerID,"rio");

    public IntakeIOReal() {
        var rollerConfigs = new TalonFXConfiguration();

        rollerConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.intakeSupplyCurrentLimit;
        rollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.intakeStatorCurrentLimit;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        rollerConfigs.TorqueCurrent.PeakForwardTorqueCurrent = SubsystemConstants.intakePeakForwardTorqueCurrent;
        rollerConfigs.TorqueCurrent.PeakReverseTorqueCurrent = SubsystemConstants.intakePeakReverseTorqueCurrent;
        rollerConfigs.TorqueCurrent.TorqueNeutralDeadband = SubsystemConstants.intakeTorqueNeutralDeadband;

        var deployConfigs = new TalonFXConfiguration();
        deployConfigs.CurrentLimits = rollerConfigs.CurrentLimits;
        deployConfigs.TorqueCurrent = rollerConfigs.TorqueCurrent;
        deployConfigs.Voltage.PeakReverseVoltage = SubsystemConstants.intakePeakReverseVoltage;
        deployConfigs.Voltage.PeakForwardVoltage = SubsystemConstants.intakePeakForwardVoltage;

        deployConfigs.MotionMagic.MotionMagicCruiseVelocity =  SubsystemConstants.intakeMotionMagicCruiseVelocity;
        deployConfigs.MotionMagic.MotionMagicAcceleration = SubsystemConstants.intakeMotionMagicAcceleration;
        deployConfigs.MotionMagic.MotionMagicJerk = SubsystemConstants.intakeMotionMagicJerk;

        deployConfigs.MotionMagic.MotionMagicJerk = SubsystemConstants.intakeMotionMagicJerk;

        var slot0 = deployConfigs.Slot0;
        slot0.kS = SubsystemConstants.intakekS;
        slot0.kV = SubsystemConstants.intakekV;
        slot0.kA = SubsystemConstants.intakekA;
        slot0.kG = SubsystemConstants.intakekG;
        slot0.kP = SubsystemConstants.intakekP;
        slot0.kI = SubsystemConstants.intakekI;
        slot0.kD = SubsystemConstants.intakekD;

        intakeRoller.getConfigurator().apply(rollerConfigs);
        intakeDeployMaster.getConfigurator().apply(deployConfigs);
        intakeDeploySlave.getConfigurator().apply(deployConfigs);

        intakeDeployMaster.setNeutralMode(NeutralModeValue.Brake);
        intakeDeploySlave.setNeutralMode(NeutralModeValue.Brake);

        intakeDeployMaster.setPosition(0.0);
        intakeDeploySlave.setPosition(0.0);

        intakeDeploySlave.setControl(new Follower(intakeDeployMaster.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.voltage = intakeRoller.getMotorVoltage().getValue().in(Units.Volts);
        inputs.position = intakeDeployMaster.getPosition().getValueAsDouble();
        inputs.deployConnected = intakeDeployMaster.isConnected() && intakeDeploySlave.isConnected();
        Logger.recordOutput("Intake/deployVoltage", intakeDeployMaster.getMotorVoltage().getValue().in(Units.Volts));

        Logger.recordOutput("Intake/masterPosition", intakeDeployMaster.getPosition().getValueAsDouble());
        Logger.recordOutput("Intake/slavePosition", intakeDeploySlave.getPosition().getValueAsDouble());
    }

    @Override
    public void setSpeed(double speed) {
        intakeRoller.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setDeployPosition(double position) {
        Logger.recordOutput("Intake/wantedPositionRotations", position);
        intakeDeployMaster.setControl(
                new MotionMagicVoltage(position)
                        .withEnableFOC(true)
                        .withSlot(0)
        );

//        intakeDeploySlave.setControl(
//                new MotionMagicVoltage(-position)
//                        .withEnableFOC(true)
//                        .withSlot(0)
//        );
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Intake/wantedVoltage", voltage);
        intakeRoller.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setDeployVoltage(double voltage) {
        intakeDeployMaster.setControl(new VoltageOut(voltage));
    }

    @Override
    public void rezero(double position) {
        intakeDeployMaster.setPosition(position);
    }

}