
package frc2025.subsystems.pivot;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import frc2025.config.SubsystemConstants;

public class PivotIONEO implements PivotIO {
    private final SparkMax pivotMaster = new SparkMax(67, SparkLowLevel.MotorType.kBrushless);
    private final SparkClosedLoopController pivotController = pivotMaster.getClosedLoopController();
    private final RelativeEncoder pivotEncoder = pivotMaster.getEncoder();

    public PivotIONEO() {
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
        pivotMaster.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private double toDegree(double rotations) {
        return 1/SubsystemConstants.pivotRotationsPerDegree*rotations;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.positionRotations = pivotEncoder.getPosition();
        inputs.positionDegrees = toDegree(inputs.positionRotations);
        inputs.positionDegreesReal = inputs.positionDegrees + SubsystemConstants.pivotZeroOffsetDegrees;

        Logger.recordOutput("Pivot/voltage", pivotMaster.getBusVoltage());
        Logger.recordOutput("Pivot/current", pivotMaster.getOutputCurrent());
    }

    @Override
    public void setPosition(double position) {
        pivotController.setReference(position, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0);
        Logger.recordOutput("Pivot/setPositionPosition", position);
    }

    @Override
    public void setVoltage(double voltage) {
        pivotMaster.setVoltage(voltage);
    }

    @Override
    public void rezero(double position) {
        pivotEncoder.setPosition(0);
    }
}
