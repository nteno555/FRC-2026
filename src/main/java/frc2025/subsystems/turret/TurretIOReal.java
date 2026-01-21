package frc2025.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class TurretIOReal implements TurretIO{
    private final TalonFX turretMotor = new TalonFX(15, "rio");

    public TurretIOReal() {
        turretMotor.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = SubsystemConstants.turretkS;
        slot0Configs.kV = SubsystemConstants.turretkV;
        slot0Configs.kP = SubsystemConstants.turretkP;
        slot0Configs.kI = SubsystemConstants.turretkI;
        slot0Configs.kD = SubsystemConstants.turretkD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = SubsystemConstants.turretCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = SubsystemConstants.turretAcceleration;
        motionMagicConfigs.MotionMagicJerk = SubsystemConstants.turretJerk;

        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.turretSupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.turretStatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = SubsystemConstants.turretPeakForwardTorqueCurrent;
        configTorque.PeakReverseTorqueCurrent = SubsystemConstants.turretPeakReverseTorqueCurrent;
        configTorque.TorqueNeutralDeadband = SubsystemConstants.turretTorqueNeutralDeadband;

        turretMotor.getConfigurator().apply(talonFXConfigs);
        turretMotor.getConfigurator().apply(configTorque);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private double toDegree(double rotations) {
        return 1/SubsystemConstants.pivotRotationsPerDegree*rotations;
    }

    private ControlRequest createControlRequest(double targetValue, boolean useVoltage) {
        double pivotPositionDegrees = toDegree(turretMotor.getPosition().getValue().in(Units.Rotation)) + SubsystemConstants.turretZeroOffsetDegrees;
        double voltageFF = -SubsystemConstants.turretkGFactor *
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
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionTicks = turretMotor.getPosition().getValue().in(Units.Rotation);
        inputs.positionDegrees = toDegree(inputs.positionTicks);
        inputs.positionDegreesReal = inputs.positionDegrees + SubsystemConstants.turretZeroOffsetDegrees;
        inputs.positionRadiansReal = inputs.positionDegrees * (Math.PI/180);

        Logger.recordOutput("Turret/voltage", turretMotor.getMotorVoltage().getValue().in(Units.Volts));
        Logger.recordOutput("Turret/current", turretMotor.getSupplyCurrent().getValue().in(Units.Amp));
    }

    @Override
    public void setPosition(double radians) {
        double position = radians * SubsystemConstants.turretRotationsPerRadian;
        turretMotor.setControl(createControlRequest(position, false));
        Logger.recordOutput("Turret/setPosPosition", position);
    }

    @Override
    public void setVoltage(double voltage) {
        turretMotor.setControl(createControlRequest(voltage, true));
    }

    @Override
    public void rezero(double radians) {
        double position = radians * SubsystemConstants.turretRotationsPerRadian;
        turretMotor.setPosition(position);
    }
}