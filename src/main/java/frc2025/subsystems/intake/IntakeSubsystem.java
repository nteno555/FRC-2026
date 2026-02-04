package frc2025.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    boolean usingVoltage = false;
    private Trigger hasPieceTrigger = (new Trigger(() -> {return inputs.current>28;})).debounce(0.1);
    private boolean intakeExtended = false;

    public IntakeSubsystem(IntakeIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        Logger.processInputs("RealOutputs/Intake", inputs);
        io.updateInputs(inputs);

        if (DriverStation.isAutonomousEnabled() && RobotState.getInstance().getWantAutoIntaking()) {
            io.setVoltage(0.0);
        }
    }

    public Command setRollerVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            usingVoltage = true;
            io.setVoltage(voltage.get());
        });
    }


    public Command setDeployVoltage(Supplier<Double> voltage) {
        return this.run(() -> io.setDeployVoltage(voltage.get()));
    }

    public Command setDeployPosition(Supplier<Double> position) {
        return this.run(() -> {
            io.setDeployPosition(position.get());
        });
    }

    public Command rezero() {
        return this.runOnce(() -> io.rezero(0.0));
    }

    public boolean getHasPiece(){
        return hasPieceTrigger.getAsBoolean();
    }
}
