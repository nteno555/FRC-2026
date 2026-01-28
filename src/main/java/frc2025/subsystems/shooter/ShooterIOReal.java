package frc2025.subsystems.shooter;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc2025.config.SubsystemConstants;

public class ShooterIOReal implements ShooterIO {
    private TalonFX flywheelMaster = new TalonFX(13, "rio");
    private TalonFX flywheelSlave = new TalonFX(17, "rio");

    public ShooterIOReal() {
		flywheelMaster.getConfigurator().apply(new TalonFXConfiguration());
		flywheelSlave.setNeutralMode(NeutralModeValue.Coast);

		var flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kS = SubsystemConstants.flywheelS;
        flywheelConfig.Slot0.kV = SubsystemConstants.flywheelV;
        flywheelConfig.Slot0.kA = SubsystemConstants.flywheelA;
        flywheelConfig.Slot0.kP = SubsystemConstants.flywheelP;
        flywheelConfig.Slot0.kI = SubsystemConstants.flywheelI;
        flywheelConfig.Slot0.kD = SubsystemConstants.flywheelD;

        var motionMagicConfigs = flywheelConfig.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)


		flywheelConfig.CurrentLimits.SupplyCurrentLimit = 100;
		flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flywheelConfig.CurrentLimits.StatorCurrentLimit = 200;
		flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		flywheelMaster.getConfigurator().apply(flywheelConfig, 0.050);
        flywheelMaster.setNeutralMode(NeutralModeValue.Coast);
        flywheelSlave.setControl(new Follower(flywheelMaster.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private ControlRequest createControlRequest(double rpm) {
        return new VelocityVoltage(rpm);
       // return new MotionMagicVelocityVoltage(rpm);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.masterVoltage = flywheelMaster.getMotorVoltage().getValueAsDouble();
        inputs.masterVelocityRPM = flywheelMaster.getVelocity().getValueAsDouble();

        inputs.slaveVoltage = flywheelSlave.getMotorVoltage().getValueAsDouble();
        inputs.slaveVelocityRPM = flywheelSlave.getVelocity().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        flywheelMaster.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        flywheelMaster.setControl(new DutyCycleOut(percentOutput));
    }

    @Override
    public void setSpeed(double speed) {
        VelocityVoltage controlRequest = new VelocityVoltage(speed)
                .withEnableFOC(true);
        flywheelMaster.setControl(controlRequest);
    }
}
