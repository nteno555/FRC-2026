package frc2025.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    double setpoint;

    public IntakeSubsystem(IntakeIO io){
        this.io = io;
        io.rezero(0.0);
    }   

    @Override
    public void periodic(){
        Logger.processInputs("RealOutputs/Intake", inputs);
        Logger.recordOutput("Intake/setpoint", setpoint);
        io.updateInputs(inputs);
    }

    public Command setRollerVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            io.setVoltage(voltage.get());
        });
    }


    public Command setDeployVoltage(Supplier<Double> voltage) {
        return this.run(() -> io.setDeployVoltage(voltage.get()));
    }

    public Command setDeployPosition(Supplier<Double> position) {
        return this.run(() -> {
            setpoint = position.get();
            io.setDeployPosition(position.get(), true);
        });
    }

    public Command oscillateIntake() {
        return this.run(() -> {
            double time = (Timer.getFPGATimestamp());

            if (time % 0.67 < 0.335) io.setDeployPosition(SubsystemConstants.intakeDeployPosition, true);
            else io.setDeployPosition(SubsystemConstants.intakeHalfPosition, true);

            io.setVoltage(SubsystemConstants.intakeRollerVoltage);
        });
    }

    public Command retractSlow() {
        return this.run(() -> {
            io.setDeployPosition(SubsystemConstants.intakeQuarterPosition, false);
//            io.setVoltage(SubsystemConstants.intakeRollerVoltage);
        });
    }

    public Command rezero() {
        return this.runOnce(() -> io.rezero(0.0));
    }

    public Command homeThenRezero() {
        return this.run(() -> {
            io.setVoltage(0.0);
            io.setDeployVoltage(-2.0);
            io.rezero(0.0);
        });
    }

    public boolean getDeployReady() {
        return inputs.position >= (SubsystemConstants.intakeDeployPosition-2.0) || !inputs.deployConnected;
    }

    public Command setIntakingAuto() {
        return this.run(() -> {
            if (inputs.deployConnected) {
                io.setDeployPosition(SubsystemConstants.intakeDeployPosition, true);
            }
            io.setVoltage(SubsystemConstants.intakeRollerVoltage);
        });
    }

    public double getDeployPosition() {
        return inputs.position;
    }

    public double getCurrent() {
        return inputs.totalCurrent;
    }
}