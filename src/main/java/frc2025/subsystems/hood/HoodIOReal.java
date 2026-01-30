package frc2025.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class HoodIOReal implements HoodIO {
    private final TalonFX hoodMaster = new TalonFX(0, "rio");

    public HoodIOReal() {
        hoodMaster.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = SubsystemConstants.hoodkS;
        slot0Configs.kV = SubsystemConstants.hoodkV;
        slot0Configs.kP = SubsystemConstants.hoodkP;
        slot0Configs.kI = SubsystemConstants.hoodkI;
        slot0Configs.kD = SubsystemConstants.hoodkD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = SubsystemConstants.hoodCruiseVelocity; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = SubsystemConstants.hoodAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = SubsystemConstants.hoodJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.hoodSupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.hoodStatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        var softLimitConfig = talonFXConfigs.SoftwareLimitSwitch;
        // Pbot hood has a 1:9 ratio
        softLimitConfig.ForwardSoftLimitThreshold = 10;
        softLimitConfig.ReverseSoftLimitThreshold = 0; 
        softLimitConfig.ForwardSoftLimitEnable = true;
        softLimitConfig.ReverseSoftLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = SubsystemConstants.hoodPeakForwardTorqueCurrent;
        configTorque.PeakReverseTorqueCurrent = SubsystemConstants.hoodPeakReverseTorqueCurrent;
        configTorque.TorqueNeutralDeadband = SubsystemConstants.hoodTorqueNeutralDeadband;

        hoodMaster.getConfigurator().apply(talonFXConfigs);
        hoodMaster.getConfigurator().apply(configTorque);
        hoodMaster.getConfigurator().apply(softLimitConfig);

        //hoodMaster.setInverted(true);
        hoodMaster.setNeutralMode(NeutralModeValue.Brake);
        rezero(0.0);
    }

    private double toRadians(double rotations) {
        return 1/SubsystemConstants.hoodRotationsPerRadian * rotations + SubsystemConstants.hoodZeroOffsetRadians;
    }

    private double toRotations(double radians) {
        return (radians - SubsystemConstants.hoodZeroOffsetRadians) * SubsystemConstants.hoodRotationsPerRadian;
    }

    private ControlRequest createControlRequest(double targetValue, boolean useVoltage) {
        double voltageFF = SubsystemConstants.hoodFF;
        if(targetValue < hoodMaster.getPosition().getValueAsDouble()) {
            voltageFF = -voltageFF;
        }

        if (useVoltage) {
            return new VoltageOut(targetValue);
        } else {
//            return new MotionMagicVoltage(targetValue)
//                    .withEnableFOC(true)
//                    //.withFeedForward(voltageFF)
//                    .withSlot(0)
//                    .withOverrideBrakeDurNeutral(true)
//                    .withLimitForwardMotion(false)
//                    .withLimitReverseMotion(false);
            return new PositionVoltage(targetValue)
                    .withFeedForward(voltageFF);
        }
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.positionRotations = hoodMaster.getPosition().getValue().in(Units.Rotation);
        inputs.positionRadians = toRadians(inputs.positionRotations);
        //inputs.positionDegreesReal = inputs.positionRadians + SubsystemConstants.hoodZeroOffsetRadians;

        Logger.recordOutput("Hood/controlRequest", hoodMaster.getAppliedControl().getName());
        Logger.recordOutput("Hood/voltage", hoodMaster.getMotorVoltage().getValue().in(Units.Volts));
        Logger.recordOutput("Hood/current", hoodMaster.getSupplyCurrent().getValue().in(Units.Amp));
    }

    @Override
    public void setPosition(double position) {
        Logger.recordOutput("Hood/wantedPosition", position);
        hoodMaster.setControl(createControlRequest(position, false));
    }


    @Override
    public void setVoltage(double voltage) {
        hoodMaster.setControl(createControlRequest(voltage, true));
    }

    @Override
    public void rezero(double position) {
        hoodMaster.setPosition(position);
    }
}
