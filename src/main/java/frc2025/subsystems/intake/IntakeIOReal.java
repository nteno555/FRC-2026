package frc2025.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc2025.RobotState;
import frc2025.subsystems.vision.VisionIO;
import frc2025.subsystems.vision.VisionPoseEstimator;
import org.littletonrobotics.junction.Logger;

import frc2025.config.SubsystemConstants;

public class IntakeIOReal implements IntakeIO {
    TalonFX intakeRoller = new TalonFX(SubsystemConstants.intakeRollerMotorID);
    TalonFX intakeDeployMaster = new TalonFX(SubsystemConstants.intakeDeployMasterMotorID);
    TalonFX intakeDeploySlave = new TalonFX(SubsystemConstants.intakeDeploySlaveMotorID);

    IntakeIOReal() {
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        rollerConfig.CurrentLimits.StatorCurrentLimit = SubsystemConstants.intakeStatorCurrentLimit;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.intakeSupplyCurrentLimit;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        rollerConfig.TorqueCurrent.PeakForwardTorqueCurrent = SubsystemConstants.intakePeakForwardTorqueCurrent;
        rollerConfig.TorqueCurrent.PeakReverseTorqueCurrent = SubsystemConstants.intakePeakReverseTorqueCurrent;
        rollerConfig.TorqueCurrent.TorqueNeutralDeadband = SubsystemConstants.intakeTorqueNeutralDeadband;
        
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();

        deployConfig.CurrentLimits = rollerConfig.CurrentLimits;
        deployConfig.TorqueCurrent = rollerConfig.TorqueCurrent;

        Slot0Configs slot0 = deployConfig.Slot0;
        slot0.kA = SubsystemConstants.intakekA;
        slot0.kP = SubsystemConstants.intakekP;
        slot0.kI = SubsystemConstants.intakekI;
        slot0.kD = SubsystemConstants.intakekD;
        slot0.kV = SubsystemConstants.intakekV;
        slot0.kS = SubsystemConstants.intakekS;
        slot0.kG = SubsystemConstants.intakekG;

        deployConfig.MotionMagic.MotionMagicAcceleration = SubsystemConstants.intakeAcceleration;
        deployConfig.MotionMagic.MotionMagicCruiseVelocity = SubsystemConstants.intakeCruiseVelocity;
        deployConfig.MotionMagic.MotionMagicJerk = SubsystemConstants.intakeJerk;

        intakeRoller.getConfigurator().apply(rollerConfig);
        intakeDeployMaster.getConfigurator().apply(deployConfig);
        intakeDeploySlave.getConfigurator().apply(deployConfig);

        intakeDeployMaster.setNeutralMode(NeutralModeValue.Brake);
        intakeDeploySlave.setNeutralMode(NeutralModeValue.Brake);

        intakeDeploySlave.setControl(new Follower(intakeDeployMaster.getDeviceID(), MotorAlignmentValue.Opposed));

        intakeDeployMaster.setPosition(SubsystemConstants.intakeDeployPosition);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.voltage = intakeRoller.getMotorVoltage().getValue().in(Units.Volts);
        inputs.position = intakeDeployMaster.getPosition().getValueAsDouble();
    }

    @Override
    public void setPosition(double position) {
        intakeDeployMaster.setPosition(position);
    }

    @Override
    public void setSpeed(double speed) {
        intakeRoller.setControl(new DutyCycleOut(speed));
    }

    @Override
    public void setVoltage(double voltage) {
        intakeRoller.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setDeplyVoltage(double voltage) {
        intakeDeployMaster.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setDeplyPosition(double position) {
        intakeDeployMaster.setControl(new PositionVoltage(position).withSlot(0));
    }

    @Override
    public void rezero(double position) {
        intakeDeployMaster.setPosition(position);
    }
}