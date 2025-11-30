package frc2025.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeMaster = new TalonFX(1,"rio");
    private double wantedVoltage = 0.0;
    private final CANrange canRange = new CANrange(2);
    public IntakeIOReal(){
        intakeMaster.getConfigurator().apply(new TalonFXConfiguration());
        var talonFXconfigs = new TalonFXConfiguration();
        talonFXconfigs.CurrentLimits.SupplyCurrentLimit = 40;
        talonFXconfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXconfigs.CurrentLimits.StatorCurrentLimit = 40;
        talonFXconfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque = new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = 40;
        configTorque.PeakReverseTorqueCurrent = 40;
        configTorque.TorqueNeutralDeadband = 1;
        intakeMaster.getConfigurator().apply(talonFXconfigs,0.05);
        intakeMaster.getConfigurator().apply(configTorque,0.05);
    }
    private ControlRequest createControlRequest(double targetValue, boolean useVoltage){
        if(useVoltage){
            return new VoltageOut(targetValue);
        } else {
            return new DutyCycleOut(targetValue);
        }
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.canRangeDistance = canRange.getDistance().getValueAsDouble();
        inputs.voltage = intakeMaster.getMotorVoltage().getValueAsDouble();
        inputs.current = intakeMaster.getTorqueCurrent().getValueAsDouble();

    }

    @Override
    public void setSpeed(double speed) {
        intakeMaster.setControl(createControlRequest(speed,false));
    }

    @Override
    public void setVoltage(double voltage) {
        intakeMaster.setControl(createControlRequest(voltage,true));
    }
}
