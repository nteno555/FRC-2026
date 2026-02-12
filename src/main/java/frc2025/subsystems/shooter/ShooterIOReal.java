package frc2025.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
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

public class ShooterIOReal implements ShooterIOIF {
    private double voltage = 0;
    private double current = 0;
    private double position = 0;

    private TalonFXConfigurator config = new TalonFXConfigurator(new DeviceIdentifier());
    private TalonFX shooterMotor = new TalonFX(1,new CANBus("rio"));
    {
        TorqueCurrentConfigs current = new TorqueCurrentConfigs();
        current.PeakForwardTorqueCurrent = 40;
        current.PeakReverseTorqueCurrent = -40;
        current.TorqueNeutralDeadband = 1;
        shooterMotor.getConfigurator().apply(current);

        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimit = 40;
        limits.StatorCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = 40;
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLowerLimit = 20;
        limits.SupplyCurrentLowerTime = 10;
        shooterMotor.getConfigurator().apply(limits);
    }

    public void periodic() {
        shooterMotor.setVoltage(this.voltage);
        shooterMotor.setPosition(this.position);
    }

    @Override public void setVoltage(double voltage) {this.voltage = voltage;}
    @Override public void setCurrent(double current) {this.current = current;}
    @Override public void setPosition(double position) {this.position = position;}

    @Override public double getVoltage() {return this.voltage;}
    @Override public double getCurrent() {return this.current;}
    @Override public double getPosition() {return this.position;}
}
