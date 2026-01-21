package frc2025.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

import frc2025.config.SubsystemConstants;

public class PivotIOReal implements PivotIO {
    private final TalonFX pivotMaster = new TalonFX(0, "rio");

    public PivotIOReal() {
        pivotMaster.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = SubsystemConstants.pivotkS;
        slot0Configs.kV = SubsystemConstants.pivotkV;
        slot0Configs.kP = SubsystemConstants.pivotkP;
        slot0Configs.kI = SubsystemConstants.pivotkI;
        slot0Configs.kD = SubsystemConstants.pivotkD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = SubsystemConstants.pivotCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = SubsystemConstants.pivotAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = SubsystemConstants.pivotJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.pivotSupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.pivotStatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = SubsystemConstants.pivotPeakForwardTorqueCurrent;
        configTorque.PeakReverseTorqueCurrent = SubsystemConstants.pivotPeakReverseTorqueCurrent;
        configTorque.TorqueNeutralDeadband = SubsystemConstants.pivotTorqueNeutralDeadband;

        pivotMaster.getConfigurator().apply(talonFXConfigs);
        pivotMaster.getConfigurator().apply(configTorque);

        //pivotMaster.setInverted(true);
        pivotMaster.setNeutralMode(NeutralModeValue.Brake);
    }

    private double toDegree(double rotations) {
        return 1/SubsystemConstants.pivotRotationsPerDegree*rotations;
    }

    private ControlRequest createControlRequest(double targetValue, boolean useVoltage) {
        double pivotPositionDegrees = toDegree(pivotMaster.getPosition().getValue().in(Units.Rotation)) + SubsystemConstants.pivotZeroOffsetDegrees;
        double voltageFF = -SubsystemConstants.pivotkGFactor *
                Math.sin(pivotPositionDegrees * Math.PI/180);

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
    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRotations = pivotMaster.getPosition().getValue().in(Units.Rotation);
        inputs.positionDegrees = toDegree(inputs.positionRotations);
        inputs.positionDegreesReal = inputs.positionDegrees + SubsystemConstants.pivotZeroOffsetDegrees;

        Logger.recordOutput("Pivot/controlRequest", pivotMaster.getAppliedControl().getName());
        Logger.recordOutput("Pivot/voltage", pivotMaster.getMotorVoltage().getValue().in(Units.Volts));
        Logger.recordOutput("Pivot/current", pivotMaster.getSupplyCurrent().getValue().in(Units.Amp));
    }

    @Override
    public void setPosition(double position) {
        Logger.recordOutput("Pivot/setPositionPosition", position);
        pivotMaster.setControl(createControlRequest(position, false));
    }

    @Override
    public void setVoltage(double voltage) {
        pivotMaster.setControl(createControlRequest(voltage, true));
    }

    @Override
    public void rezero(double position) {
        pivotMaster.setPosition(position);
    }
}
