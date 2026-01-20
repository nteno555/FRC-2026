package frc2025.subsystems.shooter;

import com.ctre.phoenix6.controls.*;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc2025.config.SubsystemConstants;

public class ShooterIOReal implements ShooterIO {
    private TalonFX lowerFlywheel = new TalonFX(18, "rio"); // Placeholders

    @SuppressWarnings("removal")
    public ShooterIOReal() {
		lowerFlywheel.getConfigurator().apply(new TalonFXConfiguration());
		lowerFlywheel.setNeutralMode(NeutralModeValue.Coast);

		//var upperConfig = new TalonFXConfiguration();
		var lowerConfig = new TalonFXConfiguration();

		// upperConfig.CurrentLimits.SupplyCurrentLimit = 40;
		// upperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		// upperConfig.CurrentLimits.StatorCurrentLimit = 100;
		// upperConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // upperConfig.Slot0.kS = SubsystemConstants.upperS;
        // upperConfig.Slot0.kV = SubsystemConstants.upperV;
        // upperConfig.Slot0.kP = SubsystemConstants.upperP;
        // upperConfig.Slot0.kI = SubsystemConstants.upperI;
        // upperConfig.Slot0.kD = SubsystemConstants.upperD;

        lowerConfig.Slot0.kS = SubsystemConstants.lowerS;
        lowerConfig.Slot0.kV = SubsystemConstants.lowerV;
        lowerConfig.Slot0.kA = SubsystemConstants.lowerA;
        lowerConfig.Slot0.kP = SubsystemConstants.lowerP;
        lowerConfig.Slot0.kI = SubsystemConstants.lowerI;
        lowerConfig.Slot0.kD = SubsystemConstants.lowerD;

        var motionMagicConfigs = lowerConfig.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)


		lowerConfig.CurrentLimits.SupplyCurrentLimit = 100;
		lowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		lowerConfig.CurrentLimits.StatorCurrentLimit = 200;
		lowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        

		lowerFlywheel.getConfigurator().apply(lowerConfig, 0.050);
        lowerFlywheel.setNeutralMode(NeutralModeValue.Coast);
    }

    private ControlRequest createControlRequest(double rpm) {
        return new VelocityVoltage(rpm);
       // return new MotionMagicVelocityVoltage(rpm);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.lowerVoltage = lowerFlywheel.getMotorVoltage().getValueAsDouble();
        inputs.lowerVelocityRPM = lowerFlywheel.getVelocity().getValueAsDouble();
        Logger.recordOutput("Shooter/lowerflywheelvel",lowerFlywheel.getVelocity().getValueAsDouble());
    }
    
    @Override
    public void setVoltage(double voltage) {
        lowerFlywheel.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setPercentOutput(double percentOutput) {
        lowerFlywheel.setControl(new DutyCycleOut(percentOutput));
    }

    @Override
    public void setSpeed(double speed) {
        //var controlRequest = createControlRequest(speed);
        VelocityVoltage controlRequest = new VelocityVoltage(speed);
        Logger.recordOutput("Shooter/controlRequestVelocity", controlRequest.Velocity);
        
        //Logger.recordOutput("Shooter/controlRequestVoltage", controlRequest.)
        lowerFlywheel.setControl(controlRequest);
    }

    @Override
    public boolean getShotReady(double wantedSpeed) {
        Logger.recordOutput("Shooter/getVelocityValAsDouble", lowerFlywheel.getVelocity().getValueAsDouble());
        return Math.abs(lowerFlywheel.getVelocity().getValueAsDouble() - wantedSpeed) < 20.0;
    }
}
