package frc2025.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import javax.swing.text.Position;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeDeployMotor = new TalonFX(SubsystemConstants.intakeDeployMasterID, CANBus.roboRIO());
    //private final TalonFX intakeDeploySlave = new TalonFX(SubsystemConstants.intakeDeploySlaveID, CANBus.roboRIO());
    private final TalonFX intakeRollerMaster = new TalonFX(SubsystemConstants.intakeRollerMasterID,CANBus.roboRIO());
    private final TalonFX intakeDeploySlave = new TalonFX(SubsystemConstants.intakeDeploySlaveID,CANBus.roboRIO());

    private final PositionVoltage voltageRequest = new PositionVoltage(0);
    private final VoltageOut voltageOutRequest = new VoltageOut(12.0);
    private final PositionTorqueCurrentFOC torqueRequest = new PositionTorqueCurrentFOC(0);

    public IntakeIOReal() {
        var rollerConfigs = new TalonFXConfiguration();

        rollerConfigs.CurrentLimits.SupplyCurrentLimit = 80;
        rollerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfigs.CurrentLimits.StatorCurrentLimit = 80;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        rollerConfigs.TorqueCurrent.PeakForwardTorqueCurrent = SubsystemConstants.intakePeakForwardTorqueCurrent;
        rollerConfigs.TorqueCurrent.PeakReverseTorqueCurrent = SubsystemConstants.intakePeakReverseTorqueCurrent;
        rollerConfigs.TorqueCurrent.TorqueNeutralDeadband = SubsystemConstants.intakeTorqueNeutralDeadband;

        var deployConfigs = new TalonFXConfiguration();
        deployConfigs.CurrentLimits.SupplyCurrentLimit = 100;
        deployConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployConfigs.CurrentLimits.StatorCurrentLimit = 100;
        deployConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        deployConfigs.TorqueCurrent = rollerConfigs.TorqueCurrent;
        deployConfigs.Voltage.PeakReverseVoltage = SubsystemConstants.intakePeakReverseVoltage;
        deployConfigs.Voltage.PeakForwardVoltage = SubsystemConstants.intakePeakForwardVoltage;

        deployConfigs.MotionMagic.MotionMagicCruiseVelocity =  SubsystemConstants.intakeMotionMagicCruiseVelocity;
        deployConfigs.MotionMagic.MotionMagicAcceleration = SubsystemConstants.intakeMotionMagicAcceleration;
        deployConfigs.MotionMagic.MotionMagicJerk = SubsystemConstants.intakeMotionMagicJerk;

        deployConfigs.MotionMagic.MotionMagicJerk = SubsystemConstants.intakeMotionMagicJerk;
        deployConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        var slot0 = deployConfigs.Slot0;
        slot0.kS = SubsystemConstants.intakekS;
        slot0.kV = SubsystemConstants.intakekV;
        slot0.kA = SubsystemConstants.intakekA;
        slot0.kG = SubsystemConstants.intakekG;
        slot0.kP = SubsystemConstants.intakekP;
        slot0.kI = SubsystemConstants.intakekI;
        slot0.kD = SubsystemConstants.intakekD;

        var slot1 = deployConfigs.Slot1;
        slot1.kS = SubsystemConstants.intakekS;
        slot1.kV = SubsystemConstants.intakekV;
        slot1.kA = SubsystemConstants.intakekA;
        slot1.kG = SubsystemConstants.intakekG;
        slot1.kP = SubsystemConstants.intakekP * 0.2;
        slot1.kI = SubsystemConstants.intakekI;
        slot1.kD = SubsystemConstants.intakekD;

        var slot2 = deployConfigs.Slot2;
        slot2.kS = 0.0;
        slot2.kV = 0.0;
        slot2.kA = 0.0;
        slot2.kP = 1.0;
        slot2.kI = 0.0;
        slot2.kD = 0.0;

        intakeRollerMaster.getConfigurator().apply(rollerConfigs);
        intakeDeploySlave.getConfigurator().apply(deployConfigs);
        intakeDeployMotor.getConfigurator().apply(deployConfigs);

        intakeDeployMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeDeployMotor.setPosition(0.0);

        intakeDeploySlave.setControl(new Follower(intakeDeployMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.voltage = intakeRollerMaster.getMotorVoltage().getValue().in(Units.Volts);
        inputs.position = intakeDeployMotor.getPosition().getValueAsDouble();
        inputs.deployConnected = intakeDeployMotor.isConnected();
//        Logger.recordOutput("Intake/deployVoltage", intakeDeployMaster.getMotorVoltage().getValue().in(Units.Volts));
//        Logger.recordOutput("Intake/deployRPS", intakeDeployMaster.getVelocity().getValueAsDouble());
//        Logger.recordOutput("Intake/masterPosition", intakeDeployMaster.getPosition().getValueAsDouble());
//        Logger.recordOutput("Intake/slavePosition", intakeDeploySlave.getPosition().getValueAsDouble());

        inputs.totalCurrent = intakeDeployMotor.getStatorCurrent().getValueAsDouble() +
                intakeRollerMaster.getStatorCurrent().getValueAsDouble() +
                intakeDeploySlave.getStatorCurrent().getValueAsDouble();

        Logger.recordOutput("Current/Intake Deploy Master", intakeDeployMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Current/Intake Deploy Slave", intakeRollerMaster.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Current/Intake Roller", intakeDeploySlave.getStatorCurrent().getValueAsDouble());
    }

    //private final Follower rollerFollower = new Follower(SubsystemConstants.intakeRollerMasterID, MotorAlignmentValue.Opposed);

    @Override
    public void setSpeed(double speed) {
        intakeRollerMaster.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setDeployPosition(double position, boolean fast) {
        if (fast) {
            intakeDeployMotor.setControl(
                    voltageRequest.withPosition(position)
                            .withEnableFOC(true)
                            .withSlot(0)
            );

//            if (intakeDeployMaster.getPosition().getValueAsDouble() > 0.5) {
//                Logger.recordOutput("Intake/deployTorque", true);
//                intakeDeployMaster.setControl(torqueRequest.withPosition(position)
//                        .withSlot(2));
//            }
//            else {
//                Logger.recordOutput("Intake/deployTorque", false);
//                intakeDeployMaster.setControl(voltageOutRequest);
//            }
        } else {
            intakeDeployMotor.setControl(
                new PositionVoltage(position)
                        .withEnableFOC(true)
                        .withSlot(1)
            );
        }
    }

    @Override
    public void setVoltage(double voltage) {
        intakeRollerMaster.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setDeployVoltage(double voltage) {
        intakeDeployMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void rezero(double position) {
        intakeDeployMotor.setPosition(position);
    }

}