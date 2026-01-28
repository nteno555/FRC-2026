package frc2025.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.Robot;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.sql.Driver;
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

    public Command setSpeed(Supplier<Double> speed) {
        return this.run(() -> {
            usingVoltage = false;
            if (speed.get() == 0.0) {
                io.setDeployPosition(0.0);
                io.setSpeed(0.0);
            }
            else {
                io.setSpeed(speed.get());
                io.setDeployPosition(SubsystemConstants.intakeDeployPosition);
            }
        });
    }

    public Command setRollerVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            usingVoltage = true;
            io.setVoltage(voltage.get());
        });
    }

    public Command setVoltage(Supplier<Double> voltage){
        return this.run(() -> {
            usingVoltage = true;
            if (voltage.get() == 0.0) {
                io.setDeployPosition(0.0);
                io.setVoltage(0.0);
            }
            else {
                io.setDeployPosition(SubsystemConstants.intakeDeployPosition);
                io.setVoltage(voltage.get());
            }
        });
    }

    public Command setIntaking(Supplier<Boolean> intake){
        return this.runEnd(() -> {
                    usingVoltage = true;

                    if (intake.get()) {
                        io.setVoltage(5.7);
                        io.setDeployPosition(SubsystemConstants.intakeDeployPosition);

                        if (Math.abs(SubsystemConstants.intakeDeployPosition - inputs.position) < 0.5) {
                            intakeExtended = true;
                        }
                        if (intakeExtended) {
                            intakeExtended = true;
                            io.setDeployVoltage(0.0);
                        }
                    }
                    else {
                        io.setVoltage(0.0);
                        intakeExtended = false;
                    }
                },
                () -> {
                    if (!DriverStation.isTeleopEnabled()) {
                        io.setVoltage(0.0);
                    }
                }
            );
    }

    public Command setOuttaking(Supplier<Boolean> intake){
        return this.run(() -> {
            usingVoltage = false;
            if(intake.get()) {
                io.setDeployPosition(SubsystemConstants.intakeDeployPosition);
                io.setSpeed(-0.3);
            }
            else {
                io.setDeployPosition(0.0);
                io.setSpeed(0);
            }
        });
    }

    public Command setDeployVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            io.setDeployVoltage(voltage.get());
        });
    }

    public Command rezero() {
        return this.runOnce(() -> io.rezero(0.0));
    }

    public Command deploy() {
        if (Math.abs(inputs.position - SubsystemConstants.intakeDeployPosition) < 0.5) {
            return this.run(() -> io.setDeployPosition(0.0));
        }
        return this.run(() -> io.setDeployPosition(SubsystemConstants.intakeDeployPosition));
    }

    public boolean getHasPiece(){
        return hasPieceTrigger.getAsBoolean();
    }
}
