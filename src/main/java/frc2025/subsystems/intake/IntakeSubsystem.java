package frc2025.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    double setpoint = 0.0;

    public IntakeSubsystem(IntakeIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        Logger.processInputs("RealOutputs/Intake", inputs);
        Logger.recordOutput("Intake/setpoint", this.setpoint);
        io.updateInputs(inputs);
    }

    public Command setRollerVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            io.setVoltage(voltage.get());
        });
    }


    public Command setDeployVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            io.setDeployVoltage(voltage.get());
            io.setVoltage(voltage.get());
        });
    }

    public Command setDeployPosition(Supplier<Double> position) {
        return this.run(() -> {
            double target = position.get();
            setpoint = target;
            if (target == SubsystemConstants.intakeRetractPosition) io.setVoltage(0.0);
            io.setDeployPosition(target, true);
        });
    }

    // private Command goTo(Supplier<Double> target) {
    // return setDeployPosition(target)
    //         .until(() -> Math.abs(inputs.position - target.get()) < 1.0)
    //         .withTimeout(1.5);
    // }

    // public Command oscillateIntake() {
    //     return Commands.sequence(
    //             goTo(() -> SubsystemConstants.intakeDeployPosition),
    //             goTo(() -> SubsystemConstants.intakeHalfPosition)
    //     ).repeatedly();
    // }

    public Command oscillateIntake() {
        Timer timer = new Timer();
        return this.run(() -> {
            boolean out = (((int) (timer.get() / 1.0)) % 2 == 0);
            double target = 0.0;
            if (out) {
                target = SubsystemConstants.intakeDeployPosition;
            } else {
                target = SubsystemConstants.intakeHalfPosition;
            }
            setpoint = target;
            io.setDeployPosition(target, true);
        }).beforeStarting(timer::restart);
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
            io.setDeployVoltage(2.0);
            io.rezero(0.0);
        });
    }

    public boolean getDeployReady() {
        return inputs.position >= (SubsystemConstants.intakeDeployPosition-2.0) || !inputs.deployConnected;
    }

    public Command setIntakingAuto() {
        return this.run(() -> {
            // if (inputs.deployConnected) {
            //     io.setDeployPosition(SubsystemConstants.intakeDeployPosition, true);
            // }
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
