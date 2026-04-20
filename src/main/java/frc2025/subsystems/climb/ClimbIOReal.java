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
    private final TalonFX climb2 = new TalonFX(SubsystemConstants.climbID2, CANBus.roboRIO());
    private final boolean NEED_DOUBLE_CLIMB = SubsystemConstants.needDoubleClimb;

    private final TalonFX tower = new TalonFX(SubsystemConstants.towerID, CANBus.roboRIO());
    private final TalonFX tower2 = new TalonFX(SubsystemConstants.towerID, CANBus.roboRIO());
    private final boolean HAS_TOWER = SubsystemConstants.hasTower;
    private final boolean NEED_DOUBLE_TOWER = SubsystemConstants.needDoubleTower;

    private final DutyCycleOut speedRequest = new DutyCycleOut(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage towerSpeedRequest = new VelocityVoltage(0).withEnableFOC(true);

    public ClimbIOReal() {
        climb.getConfigurator().apply(new TalonFXConfiguration());
        if (NEED_DOUBLE_CLIMB) climb2.getConfigurator().apply(new TalonFXConfiguration());

        var climbConfigs = new TalonFXConfiguration();
        
        climbConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.climbSupplyCurrentLimit;
        climbConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        climbConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.climbStatorCurrentLimit;
        climbConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = 100;
        configTorque.PeakReverseTorqueCurrent = 100;
        configTorque.TorqueNeutralDeadband = SubsystemConstants.climbTorqueNeutralDeadband;

        climbConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climb.getConfigurator().apply(climbConfigs, 0.050);
        climb.getConfigurator().apply(configTorque, 0.05);
        if (NEED_DOUBLE_CLIMB) climb2.getConfigurator().apply(climbConfigs, 0.050);
        if (NEED_DOUBLE_CLIMB) climb2.getConfigurator().apply(configTorque, 0.05);  

        if (NEED_DOUBLE_CLIMB) climb2.setControl(new Follower(climb.getDeviceID(), MotorAlignmentValue.Opposed));

        if (HAS_TOWER) {
            var towerConfigs = new TalonFXConfiguration();

            towerConfigs.Slot0.kS = SubsystemConstants.towerkS;
            towerConfigs.Slot0.kV = SubsystemConstants.towerkV;
            towerConfigs.Slot0.kA = SubsystemConstants.towerkA;
            towerConfigs.Slot0.kP = SubsystemConstants.towerkP;
            towerConfigs.Slot0.kI = SubsystemConstants.towerkI;
            towerConfigs.Slot0.kD = SubsystemConstants.towerkD;

            towerConfigs.Slot1.kS = SubsystemConstants.towerkSAmps;
            towerConfigs.Slot1.kV = SubsystemConstants.towerkVAmps;
            towerConfigs.Slot1.kA = SubsystemConstants.towerkAAmps;
            towerConfigs.Slot1.kP = SubsystemConstants.towerkPAmps;
            towerConfigs.Slot1.kI = SubsystemConstants.towerkIAmps;
            towerConfigs.Slot1.kD = SubsystemConstants.towerkDAmps;

            towerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            towerConfigs.CurrentLimits.SupplyCurrentLimit = 100;
            towerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
            towerConfigs.CurrentLimits.StatorCurrentLimit = 100;
            towerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

            TorqueCurrentConfigs towerConfigTorque =  new TorqueCurrentConfigs();
            towerConfigTorque.PeakForwardTorqueCurrent = 100;
            towerConfigTorque.PeakReverseTorqueCurrent = 100;
            towerConfigTorque.TorqueNeutralDeadband = 1;

            tower.getConfigurator().apply(towerConfigs, 0.050);
            tower.getConfigurator().apply(towerConfigTorque, 0.05);
            if (NEED_DOUBLE_TOWER) {
                tower2.getConfigurator().apply(towerConfigs, 0.050);
                tower2.getConfigurator().apply(towerConfigTorque, 0.05);
                tower2.setControl(new Follower(tower.getDeviceID(), MotorAlignmentValue.Opposed));
            }
        }
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.climbVoltage = climb.getMotorVoltage().getValueAsDouble();
        inputs.climbCurrent = climb.getStatorCurrent().getValueAsDouble();
        inputs.connected = climb.isConnected();

        if (NEED_DOUBLE_CLIMB) {
            inputs.climbVoltage += climb2.getMotorVoltage().getValueAsDouble();
            inputs.climbCurrent += climb2.getStatorCurrent().getValueAsDouble();
            inputs.connected = inputs.connected && climb2.isConnected();
        }

        inputs.totalCurrent = climb.getStatorCurrent().getValueAsDouble();
        if (NEED_DOUBLE_CLIMB) inputs.totalCurrent += climb2.getStatorCurrent().getValueAsDouble();
        if (HAS_TOWER) {
            inputs.totalCurrent += tower.getStatorCurrent().getValueAsDouble();
            if (NEED_DOUBLE_TOWER) inputs.totalCurrent += tower2.getStatorCurrent().getValueAsDouble();
        }

        Logger.recordOutput("Current/Climb", climb.getStatorCurrent().getValueAsDouble());
        if (NEED_DOUBLE_CLIMB) Logger.recordOutput("Current/Climb", climb2.getStatorCurrent().getValueAsDouble());
        if (HAS_TOWER) {
            Logger.recordOutput("Current/Tower", tower.getStatorCurrent().getValueAsDouble());
            if (NEED_DOUBLE_TOWER) Logger.recordOutput("Current/Tower", tower.getStatorCurrent().getValueAsDouble());
        }
        
    }

    @Override
    public void setSpeed(double climbSpeed) {
        climb.setControl(speedRequest.withOutput(climbSpeed));
        if (NEED_DOUBLE_CLIMB) climb2.setControl(speedRequest.withOutput(climbSpeed));
    }

    @Override
    public void setVoltage(double climbVoltage) {
        climb.setControl(voltageRequest.withOutput(climbVoltage));
        if (NEED_DOUBLE_CLIMB) climb2.setControl(voltageRequest.withOutput(climbVoltage));
    }

    @Override
    public void setTowerSpeed(double speed) {
        if (HAS_TOWER) {
            tower.setControl(towerSpeedRequest.withVelocity(speed * SubsystemConstants.towerFlywheelRatio));
            if (NEED_DOUBLE_TOWER) tower2.setControl(towerSpeedRequest.withVelocity(speed * SubsystemConstants.towerFlywheelRatio));
        }
    }
}