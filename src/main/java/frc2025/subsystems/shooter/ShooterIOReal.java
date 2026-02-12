package frc2025.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc2025.RobotState;
import frc2025.subsystems.vision.VisionIO;
import frc2025.subsystems.vision.VisionPoseEstimator;
import org.littletonrobotics.junction.Logger;

import frc2025.config.SubsystemConstants;

public class ShooterIOReal implements ShooterIO {
    TalonFX flywheelMaster = new TalonFX(SubsystemConstants.shooterMasterMotorID);
    TalonFX flywheelSlave = new TalonFX(SubsystemConstants.shooterSlaveMotorID);

    ShooterIOReal() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        var slot0 = config.Slot0;
        slot0.kP = SubsystemConstants.flywheelP;
        slot0.kI = SubsystemConstants.flywheelI;
        slot0.kD = SubsystemConstants.flywheelD;
        slot0.kS = SubsystemConstants.flywheelS;
        slot0.kV = SubsystemConstants.flywheelV;
        slot0.kA = SubsystemConstants.flywheelA;

        //?????

        flywheelMaster.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.position = flywheelMaster.getPosition().getValueAsDouble();
        inputs.voltage = flywheelMaster.getMotorVoltage().getValue().in(Units.Volts);
    }

    @Override
    public void setVoltage(double voltage) {
        flywheelMaster.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setCurrent(double current) {
        flywheelMaster.setControl(new VoltageOut(current));
    }

    @Override
    public void setPosition(double position) {
        flywheelMaster.setControl(new PositionVoltage(position).withSlot(0));
    }

    @Override
    public void rezero(double position) {
        flywheelMaster.setPosition(position);
    }
}
