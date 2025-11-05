package frc2025.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class ArmIOReal implements ArmIO{
    private final TalonFX armMaster = new TalonFX(41, "rio");

    public ArmIOReal() {
        armMaster.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0; //TODO TUNE
        slot0Configs.kS = SubsystemConstants.armkS;
        slot0Configs.kV = SubsystemConstants.armkV;
        slot0Configs.kP = SubsystemConstants.armkP;
        slot0Configs.kI = SubsystemConstants.armkI;
        slot0Configs.kD = SubsystemConstants.armkD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = SubsystemConstants.armCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = SubsystemConstants.armAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = SubsystemConstants.armJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.armSupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.armStatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = SubsystemConstants.armPeakForwardTorqueCurrent;
        configTorque.PeakReverseTorqueCurrent = SubsystemConstants.armPeakReverseTorqueCurrent;
        configTorque.TorqueNeutralDeadband = SubsystemConstants.armTorqueNeutralDeadband;

        armMaster.getConfigurator().apply(talonFXConfigs);
        armMaster.getConfigurator().apply(configTorque);

        armMaster.setInverted(true);
        armMaster.setNeutralMode(NeutralModeValue.Brake);
    }

    private double toDegree(double rotations) {
        return 1 / SubsystemConstants.armRotationsPerDegree * rotations;
    }

    private ControlRequest createControlRequest(double targetValue, boolean useVoltage) {
        double armPositionDegrees = toDegree(armMaster.getPosition().getValue().in(Units.Rotation)) + SubsystemConstants.armZeroOffsetDegrees;
        double voltageFF = -SubsystemConstants.armkGFactor *
                Math.sin(armPositionDegrees * Math.PI/180);
        Logger.recordOutput("Arm/voltageFF", voltageFF);

        if (useVoltage) {
            return new VoltageOut(targetValue);
        } else {
            return new MotionMagicVoltage(targetValue)
                    .withEnableFOC(true)
                    .withFeedForward(voltageFF)
                    .withSlot(0)
                    .withOverrideBrakeDurNeutral(true)
                    .withLimitForwardMotion(false)
                    .withLimitReverseMotion(false);
        }
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.positionRotations = armMaster.getPosition().getValue().in(Units.Rotation);
        inputs.positionDegrees = toDegree(inputs.positionRotations);
        inputs.positionDegreesReal = inputs.positionDegrees + SubsystemConstants.armZeroOffsetDegrees;

        Logger.recordOutput("Arm/controlRequest", armMaster.getAppliedControl().getName());
        Logger.recordOutput("Arm/voltage", armMaster.getMotorVoltage().getValue().in(Units.Volts));
        Logger.recordOutput("Arm/current", armMaster.getSupplyCurrent().getValue().in(Units.Amp));
    }

    @Override
    public void setPosition(double position) {
        Logger.recordOutput("Arm/setPositionPosition", position);
        armMaster.setControl(createControlRequest(position, false));
    }

    @Override
    public void setVoltage(double voltage) {
        armMaster.setControl(createControlRequest(voltage, true));
    }

    @Override
    public void rezero(double position) {
        armMaster.setPosition(position);
    }
<<<<<<< Updated upstream
=======

    @Override
    public void applyFeedForwardConstants(boolean loaded) {
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0 = talonFXConfigs.Slot0;

        if (loaded) {
            slot0.kS = SubsystemConstants.armkSLoaded;
            slot0.kV = SubsystemConstants.armkVLoaded;
            slot0.kP = SubsystemConstants.armkPLoaded;
            slot0.kI = SubsystemConstants.armkILoaded;
            slot0.kD = SubsystemConstants.armkDLoaded;

        } else {
            slot0.kS = SubsystemConstants.armkS;
            slot0.kV = SubsystemConstants.armkV;
            slot0.kP = SubsystemConstants.armkP;
            slot0.kI = SubsystemConstants.armkI;
            slot0.kD = SubsystemConstants.armkD;
        }

        armMaster.getConfigurator().apply(talonFXConfigs, 0.05);
    }
>>>>>>> Stashed changes
}
