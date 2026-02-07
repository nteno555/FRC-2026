package frc2025.subsystems.hood;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.RobotState;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;

public class HoodSubsystem extends SubsystemBase {
    HoodIO io;
    HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    double setpoint;
    double voltage;
    boolean usingVoltage = false;
    double voltageRamp = 0;

    public HoodSubsystem(HoodIO io) {
        io.rezero(0);
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Hood", inputs);
        Logger.recordOutput("Hood/setpoint", setpoint);
        if(DriverStation.isDisabled()) {
            setpoint = inputs.positionRotations;
        }

        io.updateInputs(inputs);
    }

    public Command setPositionRadians(Supplier<Double> positionRadians){
        return setPosition(() -> toRotations(positionRadians.get()));
    }

    public Command setPosition(Supplier<Double> position) {
        return this.run(() -> {
            setpoint = position.get();
            usingVoltage = false;
            io.setPosition(setpoint);
        });
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            usingVoltage = true;
            this.voltage = voltage.get();
            io.setVoltage(voltage.get());
        });
    }

    public Command autoAim() {
        return setPosition(() -> {
            Pose2d robotPose = RobotState.getInstance().getVisionShooterPose();
            double robotXpose = robotPose.getX();
            double robotYpose = robotPose.getY();

            boolean nearBlueUpperTrench = (robotXpose < 6.25 && robotXpose > 3.0) && robotYpose > 7.0;
            boolean nearBlueLowerTrench = (robotXpose < 6.25 && robotXpose > 3.0) && robotYpose < 1.2;
            boolean nearRedUpperTrench = (robotXpose < 13.5 && robotXpose > 10.25) && robotYpose > 7.0;
            boolean nearRedLowerTrench = (robotXpose < 13.5 && robotXpose > 10.25) && robotYpose < 1.2;

            if (nearBlueUpperTrench || nearBlueLowerTrench || nearRedUpperTrench || nearRedLowerTrench) {
                return 0.0;
            }

            Pose2d targetPose = RobotState.getInstance().getShootingWhileMovingVirtualPose();
            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
            return ShooterTreeMap.getInstance().hoodTreeMap.getInterpolated(dist);
        });
    }

    public Command autoAimFlush() {
        return setPosition(() -> ShooterTreeMap.getInstance().hoodTreeMap.getInterpolated(1.0));
    }

    public Command voltageRampCommand() {
        return this.runOnce(() -> { voltageRamp = 0.0; })
                .andThen(
                        this.run(() -> {
                            usingVoltage = true;
                            io.setVoltage(voltageRamp);
                            voltageRamp += 0.01;
                        }));
    }

    public Command rezero() {
        return this.runOnce(() -> io.rezero(0.0));
    }

    public Command homeThenRezero() {
        return setVoltage(() -> -0.5)
                .andThen(rezero());
    }

    public Command setFixed() {
        return setPosition(() -> {
            if(usingVoltage) setpoint = inputs.positionRotations;
            return setpoint;
        });
    }

    public boolean isNear(double position) {
        return Math.abs(inputs.positionRotations - position) < 2.5;
    }
    public double getPosition() { return inputs.positionRotations; }
    public double getPositionRadians() { return inputs.positionRadians; }
    public static double toRadians(double rotations) { return  -rotations / SubsystemConstants.hoodRotationsPerRadian + SubsystemConstants.hoodZeroOffsetRadians; }
    public static double toRotations(double radians) { return (SubsystemConstants.hoodZeroOffsetRadians - radians) * SubsystemConstants.hoodRotationsPerRadian; }
//    public static double toRadians(double rotations) { return io.to}
}
//