
package frc2025.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.shooter.ShooterIO;
import org.littletonrobotics.junction.Logger;

import frc2025.config.SubsystemConstants;

public class ShooterIONEO implements ShooterIO {
    private final SparkMax shooterMaster = new SparkMax(67, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax shooterSlave = new SparkMax(68, SparkLowLevel.MotorType.kBrushless);
    private final SparkClosedLoopController shooterController = shooterMaster.getClosedLoopController();
    private final RelativeEncoder shooterEncoder = shooterMaster.getEncoder();

    public ShooterIONEO() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(0.0001, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        shooterMaster.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig slaveConfig = new SparkMaxConfig();
        slaveConfig.follow(shooterMaster.getDeviceId());
        shooterSlave.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.lowerVoltage = shooterMaster.getBusVoltage();

        Logger.recordOutput("Shooter/voltage", shooterMaster.getBusVoltage());
        Logger.recordOutput("Shooter/current", shooterMaster.getOutputCurrent());
    }


    @Override
    public void setSpeed(double speed) {
        shooterController.setReference(speed, SparkBase.ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void setVoltage(double voltage) {
        shooterMaster.setVoltage(voltage);
    }
}
