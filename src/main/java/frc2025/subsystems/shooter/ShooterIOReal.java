package frc2025.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc2025.config.SubsystemConstants;

public class ShooterIOReal implements ShooterIO {
    private TalonFX upperFlywheel = new TalonFX(0, "rio"); // Placeholders
    private TalonFX lowerFlywheel = new TalonFX(0, "rio"); // Placeholders

    @SuppressWarnings("removal")
    public ShooterIOReal() {
        upperFlywheel.getConfigurator().apply(new TalonFXConfiguration());
		lowerFlywheel.getConfigurator().apply(new TalonFXConfiguration());

		upperFlywheel.setNeutralMode(NeutralModeValue.Coast);
		lowerFlywheel.setNeutralMode(NeutralModeValue.Coast);

		var upperConfig = new TalonFXConfiguration();
		var lowerConfig = new TalonFXConfiguration();

		upperConfig.CurrentLimits.SupplyCurrentLimit = 40;
		upperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		upperConfig.CurrentLimits.StatorCurrentLimit = 100;
		upperConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        upperConfig.Slot0.kS = SubsystemConstants.upperS;
        upperConfig.Slot0.kV = SubsystemConstants.upperV;
        upperConfig.Slot0.kP = SubsystemConstants.upperP;
        upperConfig.Slot0.kI = SubsystemConstants.upperI;
        upperConfig.Slot0.kD = SubsystemConstants.upperD;

        lowerConfig.Slot0.kS = SubsystemConstants.lowerS;
        lowerConfig.Slot0.kV = SubsystemConstants.lowerV;
        lowerConfig.Slot0.kP = SubsystemConstants.lowerP;
        lowerConfig.Slot0.kI = SubsystemConstants.lowerI;
        lowerConfig.Slot0.kD = SubsystemConstants.lowerD;

		lowerConfig.CurrentLimits.SupplyCurrentLimit = 40;
		lowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		lowerConfig.CurrentLimits.StatorCurrentLimit = 100;
		lowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		upperFlywheel.getConfigurator().apply(upperConfig, 0.050);
		lowerFlywheel.getConfigurator().apply(lowerConfig, 0.050);
		lowerFlywheel.setInverted(true);
		upperFlywheel.setInverted(true);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.upperVoltage = upperFlywheel.getMotorVoltage().getValueAsDouble();
        inputs.lowerVoltage = lowerFlywheel.getMotorVoltage().getValueAsDouble();
    }
    
    @Override
    public void setVoltage(double voltage) {
        upperFlywheel.setControl(new VoltageOut(voltage));
        lowerFlywheel.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setSpeed(double speed) {
        upperFlywheel.setControl(new VelocityVoltage(speed));
        lowerFlywheel.setControl(new VelocityVoltage(speed));
    }
}
