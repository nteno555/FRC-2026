package frc2025.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.subsystems.climb.ClimbIO;
import frc2025.subsystems.climb.ClimbIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
    ClimbIO io;
    ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    boolean usingVoltage = false;

    public ClimbSubsystem(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Indexer", inputs);
        io.updateInputs(inputs);
    }

}
