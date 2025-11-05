package frc2025.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class EndEffectorSubsystem extends SubsystemBase {
    EndEffectorIO io;
    EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
    boolean usingVoltage = false;
    //private Trigger hasPieceTrigger = (new Trigger(() -> {return inputs.canRangeDistance < 0.11;})).debounce(0.1);
    private Trigger hasPieceTrigger = (new Trigger(() -> {return inputs.current > 28;})).debounce(0.2);

    public EndEffectorSubsystem(EndEffectorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/EndEffector", inputs);
        io.updateInputs(inputs);
    }

    // Sets speed as a decimal from 0-1
    public Command setSpeed(Supplier<Double> speed) {
        return this.run(() -> {
            usingVoltage = false;
            io.setSpeed(speed.get());
        });
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.runOnce(() -> {
            usingVoltage = true;
            io.setVoltage(voltage.get());
        });
    }

    public Command setIntake(Supplier<Boolean> intake) {
        return this.run(() -> {
            usingVoltage = false;
            if (intake.get()) io.setSpeed(0.3);
            else io.setSpeed(0);
        });
    }

    public Command setOuttake(Supplier<Boolean> intake) {
        return this.run(() -> {
            usingVoltage = false;
            if (intake.get()) io.setSpeed(-0.3);
            else io.setSpeed(0);
        });
    }

    public Command setAuto() {
        return this.run(() -> {
            if(getHasPiece()) io.setSpeed(0.3);
            else io.setSpeed(0);
        });
    }

    public boolean getHasPiece() {
        return hasPieceTrigger.getAsBoolean();
    }
}
