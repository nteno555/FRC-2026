package frc2025.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.sql.Driver;
import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        Logger.processInputs("RealOutputs/Intake", inputs);
        Logger.recordOutput("Superstructure/IntakeDeployPosition", inputs.position);
        Logger.recordOutput("Superstructure/IntakeRollerVoltage", inputs.voltage);
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
            if (position.get() == SubsystemConstants.intakeRetractPosition) io.setVoltage(0.0);
            io.setDeployPosition(position.get());
        });
    }

    public Command oscillateIntake() {
        return this.run(() -> {
            double time = (Timer.getFPGATimestamp());

            if (time % 0.67 < 0.335) io.setDeployPosition(SubsystemConstants.intakeDeployPosition);
            else io.setDeployPosition(SubsystemConstants.intakeHalfPosition);

            io.setVoltage(SubsystemConstants.intakeRollerVoltage);
        });
    }

    public Command rezero() {
        return this.runOnce(() -> io.rezero(0.0));
    }

    public Command homeThenRezero() {
        return this.run(() -> {
            io.setVoltage(0.0);
            io.setDeployVoltage(-3.0);
            io.rezero(0.0);
        });
    }

    public boolean getDeployReady() {
        return inputs.position >= (SubsystemConstants.intakeDeployPosition-2.0) || !inputs.deployConnected;
    }

    public Command setIntakingAuto() {
        return this.run(() -> {
            io.setDeployPosition(SubsystemConstants.intakeDeployPosition);
            io.setVoltage(SubsystemConstants.intakeRollerVoltage);
        });
    }
}
