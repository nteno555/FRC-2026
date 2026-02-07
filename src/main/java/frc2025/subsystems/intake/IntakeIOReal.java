package frc2025.subsystems.intake;


import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc2025.RobotState;
import frc2025.subsystems.vision.VisionIO;
import frc2025.subsystems.vision.VisionPoseEstimator;
import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIOIF {
    private TalonFXConfigurator config = new TalonFXConfigurator(new DeviceIdentifier());

    double towerVoltage = 0;
    double towerCurrent = 0;
    double rollerVoltage = 0;
    double rollerCurrent = 0;
    double position = 0;

    public IntakeIOReal() {

    }

    @Override
    public void setTowerVoltage(double voltage) {this.towerVoltage = voltage;}
    @Override
    public void setTowerCurrent(double current) {this.towerCurrent = current;}
    @Override
    public void setRollerVoltage(double voltage) {this.rollerVoltage = voltage;}
    @Override
    public void setRollerCurrent(double current) {this.rollerCurrent = current;}
    @Override
    public void setPosition(double position) {this.position = position;}

    @Override
    public double getTowerVoltage() {return towerVoltage;}
    @Override
    public double getTowerCurrent() {return towerCurrent;}
    @Override
    public double getRollerVoltage() {return rollerVoltage;}
    @Override
    public double getRollerCurrent() {return rollerCurrent;}    
    @Override
    public double getPosition() {return position;}
}
