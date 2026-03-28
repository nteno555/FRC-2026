package frc2025.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LedSubsystem extends SubsystemBase {

    public static int firstBoardLed = 0;
    public static int lastBoardLed = 7;
    public static int firstStripLed = 8;
    public static int ledStripLength = 31;
    public static int lastStripLed = ledStripLength + 7;

    private final LedIO io;
    private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

    private LEDState currentState = LEDState.ENABLED;

    public enum LEDState {
        INDEXING,
        ENABLED
    }

    public LedSubsystem(LedIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("RealOutputs/LED", inputs);
        Logger.recordOutput("LED/state", currentState.toString());
    }

    private void applyState(LEDState state) {
        switch (state) {
            case INDEXING ->
                io.setStrobe(firstBoardLed, lastStripLed, 0, 255, 0, 50, 0);

            case ENABLED ->
                io.setSolidColor(firstBoardLed, lastStripLed, 0, 255, 0);
        }
        currentState = state;
    }

    public Command setState(LEDState state) {
        return this.runOnce(() -> applyState(state));
    }

    public Command setIndexing() {
        return setState(LEDState.INDEXING);
    }

    public Command setEnabled() {
        return setState(LEDState.ENABLED);
    }

    public Command solidColor(int r, int g, int b) {
        return this.runOnce(() -> io.setSolidColor(firstBoardLed, lastStripLed, r, g, b));
    }

    public Command off() {
        return this.runOnce(() -> io.setOff());
    }

    public LEDState getCurrentState() {
        return currentState;
    }
}