package frc2025.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Keyboard {
    private Joystick internal;

    public enum Button{
        ENTER(15),
        BACKSPACE(14),
        F1(2),
        F2(3),
        F3(4),
        F4(5),
        F5(6),
        F6(7),
        F7(8),
        F8(9),
        F9(10),
        F10(11),
        F11(12),
        F12(13),
        HOME(16),
        PGDN(17),
        PGUP(18),
        RSHIFT(20),
        LSHIFT(21),
        ESCAPE(19);


        private final int value;
        Button(int value) {
            this.value = value;
        }

        // Getter method to retrieve the value
        public int getValue() {
            return value;
        }
    }
    public Keyboard(int id) {
        internal = new Joystick(id);
    }

    public boolean getButton(Button button) {
        return internal.getRawButton(button.getValue());
    }

    public boolean getButtonPressed(Button button) {
        return internal.getRawButtonPressed(button.getValue());
    }

    public boolean getButtonReleased(Button button) {
        return internal.getRawButtonReleased(button.getValue());
    }

    public boolean getArrowUp() {
        return Math.signum(internal.getX()) > 0;
    }

    public boolean getArrowDown() {
        return Math.signum(internal.getX()) < 0;
    }

    public boolean getArrowLeft() {
        return Math.signum(internal.getZ()) < 0;
    }

    public boolean getArrowRight() {
        return Math.signum(internal.getZ()) > 0;
    }

    public int getCleanY() {
       return (int) (internal.getY() * 127 + 0.5);
    }

    public Trigger button(Button button) { return new Trigger(() -> getButton(button)); }
    public Trigger buttonPressed(Button button) { return new Trigger(() -> getButtonPressed(button)); }
    public Trigger buttonReleased(Button button) { return new Trigger(() -> getButtonReleased(button)); }
    public Trigger arrowUp() { return new Trigger(() -> getArrowUp()); }
    public Trigger arrowDown() { return new Trigger(() -> getArrowDown()); }
    public Trigger arrowLeft() { return new Trigger(() -> getArrowLeft()); }
    public Trigger arrowRight() { return new Trigger(() -> getArrowRight()); }
    //public Trigger joystickValue() { return new Trigger(() -> internal.)}
}

