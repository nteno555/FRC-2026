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

    private LEDState currentState = LEDState.DEFAULT;

    public enum LEDState {
        INDEXING,
        SHOOTING,
        PHASE_SHIFT_WARNING,
        DEFAULT
    }

    public LedSubsystem(LedIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("RealOutputs/LED", inputs);
        Logger.recordOutput("LED/state", this.currentState.toString());
    }

    private void applyState(LEDState state) {
        switch (state) {
            case INDEXING ->
                io.setStrobe(firstBoardLed, lastStripLed, 0, 255, 0, 50, 0);

            case SHOOTING ->
                io.setColorFlow(firstBoardLed, lastStripLed, 0, 100, 255, 40, 0);

            case PHASE_SHIFT_WARNING ->
                io.setStrobe(firstBoardLed, lastStripLed, 255, 255, 0, 10, 0);

            case DEFAULT ->
                io.setRainbow(firstBoardLed, lastStripLed, 1.0, 20, 0);
        }
        this.currentState = state;
    }

    public Command setState(LEDState state) {
        return this.run(() -> applyState(state));
    }

    public Command solidColor(int r, int g, int b) {
        return this.run(() -> io.setSolidColor(firstBoardLed, lastStripLed, r, g, b));
    }

    public Command off() {
        return this.run(() -> io.setOff());
    }

    public LEDState getCurrentState() {
        return this.currentState;
    }
}