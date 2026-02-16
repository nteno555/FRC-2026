package frc2025.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2025.RobotState;
import frc2025.util.Alliance;
import frc2025.util.SwerveSysidVoltageRequest;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class IntakeSubsystem extends SubsystemBase{
    // IntakeIO io;

    // public IntakeSubsystem(IntakeIO io) {
    //     this.io = io;
    // }   

    // public void periodic() {
    //     io.periodic();
    // }

    // public void rezero() {
    //     setPosition(0);
    //     io.setRollerVoltage(0);
    //     io.setTowerVoltage(0);
    // }

    // public void setRollerVoltage(double voltage) {io.setRollerVoltage(voltage);}
    // public void setDeployVoltage(double voltage) {io.setTowerVoltage(voltage);}
    // public void setPosition(double position) {io.setPosition(position);}

    // public double getRollerVoltage() {return io.getRollerVoltage();}
    // public double getDeployVoltage() {return io.getTowerVoltage();}
    // public double getPosition() {return io.getPosition();}

    // public IntakeIO getIO() {return this.io;}
}
