package frc2025.subsystems.turret;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
    TurretIO io;
    TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    double setpoint, voltage, voltageRamp;
    boolean usingVoltage = false;

    public TurretSubsystem(TurretIO io) {
        this.io = io;
        io.rezero();
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Turret", inputs);
        Logger.recordOutput("Turret/setpoint", setpoint);
        if(DriverStation.isDisabled()) {setpoint = inputs.positionTicks;}
        io.updateInputs(inputs);
    }

    public Command setPositionFieldRelative(Supplier<Double> positionRadians) {
        return setPositionSafe(() -> {
            return positionRadians.get() - RobotState.getInstance().getVisionPose().getRotation().getRadians();
        });
    }

    public Command setPositionSafe(Supplier<Double> positionRadians) {
        return this.run(() -> {
            double minAngle = Math.PI * -2;
            double maxAngle = Math.PI * 2;

            double[] candidates = {positionRadians.get(), positionRadians.get() + 2*Math.PI, positionRadians.get() - 2*Math.PI};

            double bestAngle = positionRadians.get();
            double minDistance = 9999999.0;

            for (double candidate : candidates) {
                if (candidate >= minAngle && candidate <= maxAngle) {
                    double distance = Math.abs(candidate - inputs.positionRadians);
                    if (distance < minDistance) {
                        minDistance = distance;
                        bestAngle = candidate;
                    }
                }
            }

            setpoint = io.toRotations(bestAngle);
            usingVoltage = false;
            io.setPosition(setpoint);
        });
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
        return this.run(() -> io.rezero());
    }

    public Command setFixed() {
        return setPosition(() -> {
            if(usingVoltage) setpoint = inputs.positionTicks;
            return setpoint;
        });
    }

    public double getPosition() { return inputs.positionTicks; }
    public double getPositionRadians() { return inputs.positionRadians; }
}