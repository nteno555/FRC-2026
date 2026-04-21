package frc2025.subsystems.climb;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class ClimbIOReal implements ClimbIO {
    private final TalonFX climb = new TalonFX(SubsystemConstants.climbID, CANBus.roboRIO());

    private final DutyCycleOut speedRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public ClimbIOReal() {
        climb.getConfigurator().apply(new TalonFXConfiguration());

        var climbConfigs = new TalonFXConfiguration();
        
        climbConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.climbSupplyCurrentLimit;
        climbConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        climbConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.climbStatorCurrentLimit;
        climbConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = 100;
        configTorque.PeakReverseTorqueCurrent = 100;
        configTorque.TorqueNeutralDeadband = 1;

        climbConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climb.getConfigurator().apply(climbConfigs, 0.050);
        climb.getConfigurator().apply(configTorque, 0.05);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.climbVoltage = climb.getMotorVoltage().getValueAsDouble();
        inputs.climbCurrent = climb.getStatorCurrent().getValueAsDouble();
        inputs.connected = climb.isConnected();
    }

    @Override
    public void setSpeed(double climbSpeed) {
        climb.setControl(speedRequest.withOutput(climbSpeed));
    }

    @Override
    public void setVoltage(double climbVoltage) {
        climb.setControl(voltageRequest.withOutput(climbVoltage));
    }

}