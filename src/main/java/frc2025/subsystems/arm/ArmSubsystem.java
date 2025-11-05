package frc2025.subsystems.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ArmSubsystem extends SubsystemBase {
    ArmIO io;
    ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    double setpoint;
    double voltage;
    boolean usingVoltage = false;
    double voltageRamp = 0;

    public ArmSubsystem(ArmIO io) {
        io.rezero(0);
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Arm", inputs);
        Logger.recordOutput("Arm/setpoint", setpoint);
        if(DriverStation.isDisabled()) {
            setpoint = inputs.positionRotations;
        }

        io.updateInputs(inputs);
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
        return this.run(() -> io.rezero(0.0));
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
    public double getRealAngle() { return inputs.positionDegreesReal; }
<<<<<<< Updated upstream
=======

    private boolean hasGamePiece = false;

    public void setHasGamePiece(boolean loaded) {
        this.hasGamePiece = loaded;
        /*if (io instanceof ArmIOReal) {
            ArmIOReal realIO = (ArmIOReal) realIO;
            realIO.applyFeedForwardConstants(loaded);
        }*/
        io.applyFeedForwardConstants(loaded);
    }

    public boolean isArmLoaded() {
        return hasGamePiece;
    }

>>>>>>> Stashed changes
}
