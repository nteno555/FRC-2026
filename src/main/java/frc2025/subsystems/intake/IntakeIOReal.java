package frc2025.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import frc2025.config.SubsystemConstants;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeDeploy = new TalonFX(67, "rio");
    private final TalonFX intakeRoller = new TalonFX(18,"rio");
    private double wantedVoltage = 0.0;
    public IntakeIOReal(){
        intakeDeploy.getConfigurator().apply(new TalonFXConfiguration());
        intakeRoller.getConfigurator().apply(new TalonFXConfiguration());
        var talonFXconfigs = new TalonFXConfiguration();
        talonFXconfigs.CurrentLimits.SupplyCurrentLimit = 40;
        talonFXconfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXconfigs.CurrentLimits.StatorCurrentLimit = 40;
        talonFXconfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque = new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = 40;
        configTorque.PeakReverseTorqueCurrent = 40;
        configTorque.TorqueNeutralDeadband = 1;
        intakeDeploy.getConfigurator().apply(talonFXconfigs,0.05);
        intakeDeploy.getConfigurator().apply(configTorque,0.05);
        intakeRoller.getConfigurator().apply(talonFXconfigs,0.05);
        intakeRoller.getConfigurator().apply(configTorque,0.05);
    }
    private ControlRequest createControlRequest(double targetValue, boolean useVoltage){
        if(useVoltage){
            return new VoltageOut(targetValue);
        } else {
            return new DutyCycleOut(targetValue);
        }
    }
    private double toDegree(double rotations) {
        return 1/SubsystemConstants.intakeRotationsPerDegree*rotations;
    }

    private ControlRequest createDeployControlRequest(double targetValue, boolean useVoltage) {
        double intakePositionDegrees = toDegree(intakeDeploy.getPosition().getValue().in(Units.Rotation)) + SubsystemConstants.intakeZeroOffsetDegrees;
        double voltageFF = -SubsystemConstants.intakekGFactor *
                Math.sin(intakePositionDegrees * Math.PI/180);

        if (useVoltage) {
            return new VoltageOut(targetValue);
        } else {
            return new MotionMagicVoltage(targetValue)
                    .withEnableFOC(true)
                    .withFeedForward(voltageFF)
                    .withSlot(0)
                    .withOverrideBrakeDurNeutral(true)
                    .withLimitForwardMotion(false)
                    .withLimitReverseMotion(false);
        }
    }
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.voltage = intakeRoller.getMotorVoltage().getValueAsDouble();
        inputs.current = intakeRoller.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public void setSpeed(double speed) {
        if (speed == 0.0) intakeDeploy.setControl(createDeployControlRequest(0.0, false));
        else intakeDeploy.setControl(createDeployControlRequest(SubsystemConstants.intakeDeployPosition, false));
        intakeRoller.setControl(createControlRequest(speed,false));
    }

    @Override
    public void setVoltage(double voltage) {
        if (voltage == 0.0) intakeDeploy.setControl(createDeployControlRequest(0.0, true));
        else intakeDeploy.setControl(createDeployControlRequest(SubsystemConstants.intakeDeployPosition, true));
        intakeRoller.setControl(createControlRequest(voltage,true));
    }

}
