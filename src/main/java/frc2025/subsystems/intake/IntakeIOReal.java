package frc2025.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeDeployMaster = new TalonFX(SubsystemConstants.intakeDeployMasterID, CANBus.roboRIO());
    private final TalonFX intakeDeploySlave = new TalonFX(SubsystemConstants.intakeDeploySlaveID, CANBus.roboRIO());
    private final TalonFX intakeRoller = new TalonFX(SubsystemConstants.intakeRollerID,CANBus.roboRIO());

    public IntakeIOReal() {
        var rollerConfigs = new TalonFXConfiguration();

        rollerConfigs.CurrentLimits.SupplyCurrentLimit = 80;
        rollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfigs.CurrentLimits.StatorCurrentLimit = 80;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        rollerConfigs.TorqueCurrent.PeakForwardTorqueCurrent = SubsystemConstants.intakePeakForwardTorqueCurrent;
        rollerConfigs.TorqueCurrent.PeakReverseTorqueCurrent = SubsystemConstants.intakePeakReverseTorqueCurrent;
        rollerConfigs.TorqueCurrent.TorqueNeutralDeadband = SubsystemConstants.intakeTorqueNeutralDeadband;

        var deployConfigs = new TalonFXConfiguration();
        deployConfigs.CurrentLimits.SupplyCurrentLimit = 120;
        deployConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployConfigs.CurrentLimits.StatorCurrentLimit = 120;
        deployConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        deployConfigs.TorqueCurrent = rollerConfigs.TorqueCurrent;
        deployConfigs.Voltage.PeakReverseVoltage = SubsystemConstants.intakePeakReverseVoltage;
        deployConfigs.Voltage.PeakForwardVoltage = SubsystemConstants.intakePeakForwardVoltage;

        deployConfigs.MotionMagic.MotionMagicCruiseVelocity =  SubsystemConstants.intakeMotionMagicCruiseVelocity;
        deployConfigs.MotionMagic.MotionMagicAcceleration = SubsystemConstants.intakeMotionMagicAcceleration;
        deployConfigs.MotionMagic.MotionMagicJerk = SubsystemConstants.intakeMotionMagicJerk;

        deployConfigs.MotionMagic.MotionMagicJerk = SubsystemConstants.intakeMotionMagicJerk;
        deployConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        var slot0 = deployConfigs.Slot0;
        slot0.kS = SubsystemConstants.intakekS;
        slot0.kV = SubsystemConstants.intakekV;
        slot0.kA = SubsystemConstants.intakekA;
        slot0.kG = SubsystemConstants.intakekG;
        slot0.kP = SubsystemConstants.intakekP;
        slot0.kI = SubsystemConstants.intakekI;
        slot0.kD = SubsystemConstants.intakekD;

        var slot1 = deployConfigs.Slot1;
        slot1.kS = SubsystemConstants.intakekS;
        slot1.kV = SubsystemConstants.intakekV;
        slot1.kA = SubsystemConstants.intakekA;
        slot1.kG = SubsystemConstants.intakekG;
        slot1.kP = SubsystemConstants.intakekP * 0.2;
        slot1.kI = SubsystemConstants.intakekI;
        slot1.kD = SubsystemConstants.intakekD;

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
        Logger.recordOutput("Current/intakeRoller", intakeRoller.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Current/intakeDeployMaster", intakeDeployMaster.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Current/intakeDeploySlave", intakeDeploySlave.getStatorCurrent().getValueAsDouble());
//        Logger.recordOutput("Intake/deployVoltage", intakeDeployMaster.getMotorVoltage().getValue().in(Units.Volts));
//        Logger.recordOutput("Intake/deployRPS", intakeDeployMaster.getVelocity().getValueAsDouble());
//        Logger.recordOutput("Intake/masterPosition", intakeDeployMaster.getPosition().getValueAsDouble());
//        Logger.recordOutput("Intake/slavePosition", intakeDeploySlave.getPosition().getValueAsDouble());
    }

    @Override
    public void setSpeed(double speed) {
        intakeRoller.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setDeployPosition(double position, boolean fast) {
        if (fast) {
            intakeDeployMaster.setControl(
                    new PositionVoltage(position)
                            .withEnableFOC(true)
                            .withSlot(0)
            );
        } else {
            intakeDeployMaster.setControl(
                new PositionVoltage(position)
                        .withEnableFOC(true)
                        .withSlot(1)
            );
        }
    }

    @Override
    public void setVoltage(double voltage) {
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