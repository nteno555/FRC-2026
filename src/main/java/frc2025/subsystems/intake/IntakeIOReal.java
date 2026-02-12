package frc2025.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
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

public class IntakeIOReal implements IntakeIOIF {
    private double towerVoltage = 0;
    private double towerCurrent = 0;
    private double rollerVoltage = 0;
    private double rollerCurrent = 0;
    private double position = 0;

    private TalonFX towerMotor = new TalonFX(0,new CANBus("rio"));
    {
        TorqueCurrentConfigs current = new TorqueCurrentConfigs();
        current.PeakForwardTorqueCurrent = 40;
        current.PeakReverseTorqueCurrent = -40;
        current.TorqueNeutralDeadband = 1;
        towerMotor.getConfigurator().apply(current);

        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimit = 40;
        limits.StatorCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = 40;
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLowerLimit = 20;
        limits.SupplyCurrentLowerTime = 10;
        towerMotor.getConfigurator().apply(limits);
    }
    private TalonFX rollerMotor = new TalonFX(2,new CANBus("rio"));
    {        
        TorqueCurrentConfigs current = new TorqueCurrentConfigs();
        current.PeakForwardTorqueCurrent = 40;
        current.PeakReverseTorqueCurrent = -40;
        current.TorqueNeutralDeadband = 1;
        rollerMotor.getConfigurator().apply(current);

        CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
        limits.StatorCurrentLimit = 40;
        limits.StatorCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = 40;
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLowerLimit = 20;
        limits.SupplyCurrentLowerTime = 10;
        rollerMotor.getConfigurator().apply(limits);
    }
    private TalonFXConfigurator config = new TalonFXConfigurator(new DeviceIdentifier());

    public IntakeIOReal() {

    }

    public void periodic() {
        towerMotor.setVoltage(this.towerVoltage);
        towerMotor.setPosition(this.position);
        rollerMotor.setVoltage(this.rollerVoltage);
        rollerMotor.setPosition(this.position);
        config.setPosition(this.position);
    }

    @Override public void setTowerVoltage(double voltage) {this.towerVoltage = voltage;}
    @Override public void setTowerCurrent(double current) {this.towerCurrent = current;}
    @Override public void setRollerVoltage(double voltage) {this.rollerVoltage = voltage;}
    @Override public void setRollerCurrent(double current) {this.rollerCurrent = current;}
    @Override public void setPosition(double position) {this.position = position;}

    @Override public double getTowerVoltage() {return towerVoltage;}
    @Override public double getTowerCurrent() {return towerCurrent;}
    @Override public double getRollerVoltage() {return rollerVoltage;}
    @Override public double getRollerCurrent() {return rollerCurrent;}    
    @Override public double getPosition() {return position;}

    public TalonFX getTowerMotor() {return this.towerMotor;}
    public TalonFX getRollerMotor() {return this.rollerMotor;}
    public TalonFXConfigurator getConfig() {return this.config;}
}
