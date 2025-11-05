package frc2025.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOReal implements ElevatorIO {
    //private final double voltageFF = 0.4;
    public TalonFX elevatorMaster = new TalonFX(17, "rio");
    public TalonFX elevatorSlave = new TalonFX(15, "rio");

    public ElevatorIOReal() {
        elevatorMaster.getConfigurator().apply(new TalonFXConfiguration());
        elevatorSlave.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = SubsystemConstants.elevatorkS; // todo: move to separate config file along with other constants
        slot0Configs.kV = SubsystemConstants.elevatorkV;
        slot0Configs.kA = SubsystemConstants.elevatorkA;
        slot0Configs.kG = SubsystemConstants.elevatorkG;
        slot0Configs.kP = SubsystemConstants.elevatorkP;
        slot0Configs.kI = SubsystemConstants.elevatorkI;
        slot0Configs.kD = SubsystemConstants.elevatorkD;
        //slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = SubsystemConstants.elevatorCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = SubsystemConstants.elevatorAcceleration;
        motionMagicConfigs.MotionMagicJerk = SubsystemConstants.elevatorJerk;

        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.elevatorSupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.elevatorStatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = SubsystemConstants.elevatorPeakForwardTorqueCurrent;
        configTorque.PeakReverseTorqueCurrent = SubsystemConstants.elevatorPeakReverseTorqueCurrent;
        configTorque.TorqueNeutralDeadband = SubsystemConstants.elevatorTorqueNeutralDeadband;

        elevatorMaster.getConfigurator().apply(talonFXConfigs, 0.050);
        elevatorMaster.getConfigurator().apply(configTorque, 0.05);
        elevatorMaster.setNeutralMode(NeutralModeValue.Brake);
        elevatorMaster.setInverted(true);

        elevatorSlave.getConfigurator().apply(talonFXConfigs, 0.050);
        elevatorSlave.getConfigurator().apply(configTorque, 0.05);
        elevatorSlave.setNeutralMode(NeutralModeValue.Brake);
        elevatorSlave.setInverted(false);

        elevatorSlave.setControl(new Follower(elevatorMaster.getDeviceID(), true));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotations = elevatorMaster.getPosition().getValue().in(Units.Rotation);
        inputs.voltage = elevatorMaster.getMotorVoltage().getValue().in(Units.Volts);
        Logger.recordOutput("Elevator/position", inputs.positionRotations);
        Logger.recordOutput("Elevator/voltage", elevatorMaster.getMotorVoltage().getValue().in(Units.Volts));
    }

    @Override
    public void setPosition(double meters) {
        elevatorMaster.setControl(createControlRequest(meters, false));
    }

    @Override
    public void setPositionSlow(double meters) {
        //elevatorMaster.setControl(createControlRequestSlow(meters, false));
        //elevatorMaster.setControl(new VoltageOut())
    }

    @Override
    public void setVoltage(double voltage) {
        elevatorMaster.setControl(createControlRequest(voltage, true));
    }

    @Override
    public void setCurrent(double amps) {

    }

    @Override
    public void rezero(double position) {
        elevatorMaster.setPosition(position);
    }

<<<<<<< Updated upstream
=======
    @Override
    public void applyFeedForwardConstants(boolean loaded) {
        var talonFXConfigs = new TalonFXConfiguration();
        var slot0 = talonFXConfigs.Slot0;
        if (loaded) {
            slot0.kS = SubsystemConstants.elevatorkSloaded;
            slot0.kV = SubsystemConstants.elevatorkVloaded;
            slot0.kA = SubsystemConstants.elevatorkAloaded;
            slot0.kG = SubsystemConstants.elevatorkGloaded;
            slot0.kP = SubsystemConstants.elevatorkPloaded;
            slot0.kI = SubsystemConstants.elevatorkIloaded;
            slot0.kD = SubsystemConstants.elevatorkDloaded;
        } else {
            slot0.kS = SubsystemConstants.elevatorkS;
            slot0.kV = SubsystemConstants.elevatorkV;
            slot0.kA = SubsystemConstants.elevatorkA;
            slot0.kG = SubsystemConstants.elevatorkG;
            slot0.kP = SubsystemConstants.elevatorkP;
            slot0.kI = SubsystemConstants.elevatorkI;
            slot0.kD = SubsystemConstants.elevatorkD;
        }
        elevatorMaster.getConfigurator().apply(talonFXConfigs, 0.05);
        elevatorSlave.getConfigurator().apply(talonFXConfigs, 0.05);
    }

>>>>>>> Stashed changes
    private ControlRequest createControlRequest(double targetValue, boolean useVoltage) {
        if (useVoltage) {
            return new VoltageOut(targetValue);
        } else {
            return new MotionMagicVoltage(targetValue)
                    .withEnableFOC(true)
                    //.withFeedForward(voltageFF)
                    .withSlot(0)
                    .withOverrideBrakeDurNeutral(true)
                    .withLimitForwardMotion(false)
                    .withLimitReverseMotion(false);
        }
    }

    private ControlRequest createControlRequestSlow(double targetValue, boolean useVoltage) {
        if (useVoltage) {
            return new VoltageOut(targetValue);
        } else {
            return new DynamicMotionMagicVoltage(0, 100, 50, 1000)
                    .withEnableFOC(true)
                    //.withFeedForward(voltageFF)
                    .withSlot(0)
                    .withOverrideBrakeDurNeutral(true)
                    .withLimitForwardMotion(false)
                    .withLimitReverseMotion(false);
        }
    }
}
