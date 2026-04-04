package frc2025.subsystems.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberIOReal implements ClimberIO{
    private final TalonFX climber = new TalonFX(SubsystemConstants.climberID, CANBus.roboRIO());

    public ClimberIOReal() {
        var climberConfig = new TalonFXConfiguration();

        climberConfig.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.climberSupplyCurrentLimit;
        climberConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        climberConfig.CurrentLimits.StatorCurrentLimit = SubsystemConstants.climberStatorCurrentLimit;
        climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        climberConfig.MotionMagic.MotionMagicCruiseVelocity = SubsystemConstants.climberMotionMagicCruiseVelocity;
        climberConfig.MotionMagic.MotionMagicAcceleration = SubsystemConstants.climberMotionMagicAcceleration;
        climberConfig.MotionMagic.MotionMagicJerk = SubsystemConstants.climberMotionMagicJerk;

        var slot0 = climberConfig.Slot0;
        slot0.kS = SubsystemConstants.climberkS;
        slot0.kV = SubsystemConstants.climberkV;
        slot0.kA = SubsystemConstants.climberkA;
        slot0.kP = SubsystemConstants.climberkP;
        slot0.kI = SubsystemConstants.climberkI;
        slot0.kD = SubsystemConstants.climberkD;

        climber.getConfigurator().apply(climberConfig);
        climber.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.rotations = climber.getPosition().getValue().in(Units.Rotations);
        inputs.position = inputs.rotations * SubsystemConstants.climbersMetersPerRotation;
    }

    @Override
    public void setPosition(double position) {
        Logger.recordOutput("Climber/wantedPosition", position);

        // might be other sign depending on motor
        double voltageFF = position < climber.getPosition().getValueAsDouble() ? -SubsystemConstants.climberFF : SubsystemConstants.climberFF;
        climber.setControl(new MotionMagicVoltage(position)
                .withEnableFOC(true)
                .withFeedForward(voltageFF)
                .withSlot(0)
                .withOverrideBrakeDurNeutral(true)
                .withLimitForwardMotion(false)
                .withLimitReverseMotion(false));
    }

    @Override
    public void setVoltage(double voltage) {
        climber.setControl(new VoltageOut(voltage));
    }

    @Override
    public void rezero() {
        climber.setPosition(0.0);
    }

    @Override
    public double toRotations(double position) {
        return position * (1/SubsystemConstants.climbersMetersPerRotation);
    }

    @Override
    public double toPositionMeters(double rotations) {
        return rotations * SubsystemConstants.climbersMetersPerRotation;
    }
}
