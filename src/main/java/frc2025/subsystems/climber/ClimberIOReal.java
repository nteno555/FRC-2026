package frc2025.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue; 




public class ClimberIOReal implements ClimberIO{
     private final TalonFX climber = new TalonFX(18, "rio");

    public ClimberIOReal() {
        climber.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();
        
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 40;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 100;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = 40;
        configTorque.PeakReverseTorqueCurrent = 40;
        configTorque.TorqueNeutralDeadband = 1;


        climber.getConfigurator().apply(talonFXConfigs, 0.050);
        climber.getConfigurator().apply(configTorque, 0.05);

        climber.setNeutralMode(NeutralModeValue.Brake);


    }


    private ControlRequest createControlRequest(double targetValue, boolean useVoltage) {
        if (useVoltage) {
            return new VoltageOut(targetValue);
        } else {
            return new DutyCycleOut(targetValue);
        }
    }
    
    public void updateInputs(ClimberIOInputs inputs){
        
    }

    public void setSpeed(double speed){
        climber.setControl(createControlRequest(speed, false));
    }

    public void setVoltage(double voltage){
        climber.setControl(createControlRequest(voltage, true));
    }
}
