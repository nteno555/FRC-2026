package frc2025.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX indexer = new TalonFX(14, "rio");
    private final CANrange canRange = new CANrange(15);
    // private double wantedVoltage = 0.0;

    public IndexerIOReal() {
        indexer.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();
        
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 40;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = 40;
        configTorque.PeakReverseTorqueCurrent = 40;
        configTorque.TorqueNeutralDeadband = 1;


        indexer.getConfigurator().apply(talonFXConfigs, 0.050);
        indexer.getConfigurator().apply(configTorque, 0.05);

    }




    private ControlRequest createControlRequest(double targetValue, boolean useVoltage) {
        if (useVoltage) {
            return new VoltageOut(targetValue);
        } else {
            return new DutyCycleOut(targetValue);
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // TODO: not sure if this is needed
        inputs.canRangeDistance = canRange.getDistance().getValueAsDouble();
        inputs.voltage = indexer.getMotorVoltage().getValueAsDouble();
        inputs.current = indexer.getTorqueCurrent().getValueAsDouble();

        //Logger.recordOutput("Indexer/wantedVoltage", wantedVoltage);
    }

    @Override
    public void setSpeed(double speed) {
        indexer.setControl(createControlRequest(speed, false));
    }

    @Override
    public void setVoltage(double voltage) {
        indexer.setControl(createControlRequest(voltage, true));
        //wantedVoltage = voltage;
    }
}
