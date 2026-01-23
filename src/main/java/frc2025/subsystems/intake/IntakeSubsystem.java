package frc2025.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    boolean usingVoltage = false;
    private Trigger hasPieceTrigger = (new Trigger(() -> {return inputs.current>28;})).debounce(0.1);
    public IntakeSubsystem(IntakeIO io){
        this.io = io;
    }
    @Override
    public void periodic(){
        Logger.processInputs("RealOutputs/Intake",inputs);
        io.updateInputs(inputs);
    }

    public Command setSpeed(Supplier<Double> speed){
        return this.run(() -> {
            usingVoltage = false;
            if (speed.get() == 0.0) {
                io.setZeroPosition();
                io.setSpeed(0.0);
            }
            else {
                io.setSpeed(speed.get());
                io.setDeployPosition();
            }
        });
    }

    public Command setVoltage(Supplier<Double> voltage){
        return this.runOnce(() -> {
            usingVoltage = true;
            if (voltage.get() == 0.0) {
                io.setZeroPosition();
                io.setVoltage(0.0);
            }
            else {
                io.setDeployPosition();
                io.setVoltage(voltage.get());
            }
        });
    }

    public Command setIntake(Supplier<Boolean> intake){
        return this.run(() -> {
            usingVoltage = false;
            if(intake.get()) {
                io.setSpeed(0.3);
                io.setDeployPosition();
            }
            else {
                io.setZeroPosition();
                io.setSpeed(0);
            }
        });
    }

    public Command setOuttake(Supplier<Boolean> intake){
        return this.run(() -> {
            usingVoltage = false;
            if(intake.get()) {
                io.setDeployPosition();
                io.setSpeed(-0.3);
            }
            else {
                io.setZeroPosition();
                io.setSpeed(0);
            }
        });
    }

    public boolean getHasPiece(){
        return hasPieceTrigger.getAsBoolean();
    }


}
