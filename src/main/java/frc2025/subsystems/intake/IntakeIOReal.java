package frc2025.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.Units;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeDeployMaster = new TalonFX(67, "rio");
    private final TalonFX intakeDeploySlave = new TalonFX(68, "rio");
    private final TalonFX intakeRoller = new TalonFX(11,"rio");

    public IntakeIOReal() {
        var rollerConfigs = new TalonFXConfiguration();

        rollerConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        rollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfigs.CurrentLimits.StatorCurrentLimit = 40;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        rollerConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        rollerConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        rollerConfigs.TorqueCurrent.TorqueNeutralDeadband = 1;

        var deployConfigs = new TalonFXConfiguration();
        deployConfigs.CurrentLimits = rollerConfigs.CurrentLimits;
        deployConfigs.TorqueCurrent = rollerConfigs.TorqueCurrent;

        var slot0 = deployConfigs.Slot0;
        slot0.kS = SubsystemConstants.intakekS;
        slot0.kV = SubsystemConstants.intakekV;
        slot0.kA = SubsystemConstants.intakekA;
        slot0.kG = SubsystemConstants.intakekG;
        slot0.kP = SubsystemConstants.intakekP;
        slot0.kI = SubsystemConstants.intakekI;
        slot0.kD = SubsystemConstants.intakekD;

        deployConfigs.MotionMagic.MotionMagicCruiseVelocity = SubsystemConstants.intakeCruiseVelocity;
        deployConfigs.MotionMagic.MotionMagicAcceleration = SubsystemConstants.intakeAcceleration;
        deployConfigs.MotionMagic.MotionMagicJerk = SubsystemConstants.intakeJerk;

        intakeRoller.getConfigurator().apply(rollerConfigs);
        intakeDeployMaster.getConfigurator().apply(deployConfigs);
        intakeDeploySlave.getConfigurator().apply(deployConfigs);

        intakeDeploySlave.setControl(new Follower(intakeDeployMaster.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private ControlRequest createControlRequest(double target, boolean useVoltage) {
//        double voltageFF = -SubsystemConstants.intakekGFactor *
//                Math.sin(intakePositionDegrees * Math.PI/180);

        if (useVoltage) {
            return new VoltageOut(target);
        } else {
            return new MotionMagicVoltage(target)
                    .withEnableFOC(true)
                    .withSlot(0)
                    .withOverrideBrakeDurNeutral(true)
                    .withLimitForwardMotion(false)
                    .withLimitReverseMotion(false);
        }
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.voltage = intakeRoller.getMotorVoltage().getValue().in(Units.Volts);
        inputs.position = intakeDeployMaster.getPosition().getValueAsDouble();
        Logger.recordOutput("Intake/positionRotations", intakeDeployMaster.getPosition().getValue().in(Units.Rotations));
        Logger.recordOutput("Intake/positionMeters", inputs.position);
        Logger.recordOutput("Intake/voltage", inputs.voltage);
    }

    @Override
    public void setSpeed(double speed) {
        intakeRoller.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setDeployPosition() {
        intakeDeployMaster.setControl(createControlRequest(SubsystemConstants.intakeDeployPosition, false));
    }

    @Override
    public void setZeroPosition() {
        intakeDeployMaster.setControl(createControlRequest(0.0, false));
    }

    @Override
    public void setVoltage(double voltage) {
        intakeRoller.setControl(new VoltageOut(voltage));
        if (voltage == 0.0) intakeDeployMaster.setControl(createControlRequest(0.0, true));
    }

}