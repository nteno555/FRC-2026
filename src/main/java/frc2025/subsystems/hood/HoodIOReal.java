package frc2025.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.Units;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class HoodIOReal implements HoodIO {
    private final TalonFX hood = new TalonFX(SubsystemConstants.hoodID, "rio");

    public HoodIOReal() {
        hood.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = SubsystemConstants.hoodkS;
        slot0Configs.kV = SubsystemConstants.hoodkV;
        slot0Configs.kP = SubsystemConstants.hoodkP;
        slot0Configs.kI = SubsystemConstants.hoodkI;
        slot0Configs.kD = SubsystemConstants.hoodkD;
        slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.hoodSupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.hoodStatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        var softLimitConfig = talonFXConfigs.SoftwareLimitSwitch;
        softLimitConfig.ForwardSoftLimitThreshold = SubsystemConstants.hoodForwardSoftLimit;
        softLimitConfig.ReverseSoftLimitThreshold = SubsystemConstants.hoodReverseSoftLimit;
        softLimitConfig.ForwardSoftLimitEnable = false;
        softLimitConfig.ReverseSoftLimitEnable = false;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = SubsystemConstants.hoodPeakForwardTorqueCurrent;
        configTorque.PeakReverseTorqueCurrent = SubsystemConstants.hoodPeakReverseTorqueCurrent;
        configTorque.TorqueNeutralDeadband = SubsystemConstants.hoodTorqueNeutralDeadband;

        hood.getConfigurator().apply(talonFXConfigs);
        hood.getConfigurator().apply(configTorque);
        hood.getConfigurator().apply(softLimitConfig);

        //hoodMaster.setInverted(true);
        hood.setNeutralMode(NeutralModeValue.Brake);
        rezero(0.0);
    }

    private double toRadians(double rotations) {
        return HoodSubsystem.toRadians(rotations);
    }

    private double toRotations(double radians) {
        return HoodSubsystem.toRotations(radians);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRotations = hood.getPosition().getValue().in(Units.Rotation);
        inputs.positionRadians = toRadians(inputs.positionRotations);
        inputs.positionDegreesReal = Math.toDegrees(inputs.positionRadians);
    }

    @Override
    public void setPosition(double position) {
        Logger.recordOutput("Hood/wantedPosition", position);
        Logger.recordOutput("Hood/setPositionHeartbeat", Math.random());

        if (Math.abs(position - hood.getPosition().getValueAsDouble()) < 0.02) return;

//        double voltageFF = position < hood.getPosition().getValueAsDouble() ? -SubsystemConstants.hoodFF : SubsystemConstants.hoodFF;
        hood.setControl(new PositionVoltage(position)
                .withEnableFOC(true));
    }

    @Override
    public void setVoltage(double voltage) {
        Logger.recordOutput("Hood/setVoltageHeartbeat", Math.random());
        Logger.recordOutput("Hood/wantedVoltage", voltage);
        hood.setControl(new VoltageOut(voltage));
    }

    @Override
    public void rezero(double position) {
        hood.setPosition(0.0);
    }
}
