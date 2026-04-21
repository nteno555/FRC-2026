/* 
 * Aaron's Note
 * 
 * The new ubot might be linear climb like "Jack in the bot" does. 
 * It abandon the original spinning climb, and no tower.
 * 
 * The new climb might be one motor.
 * (I don't know if the build team would like to do two motors)
 * 
 * Use the voltage to control the speed of the climb and 
 * compatitive to the planning bot as much as possible. 
 * 
 * Todo:
 * (done) 1. View the code and delete/add new functions/variable to it
 * (done) 2. Try to make a double motors system to spare.
 * 
 */

package frc2025.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.climb.ClimbIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;


public class ClimbSubsystem extends SubsystemBase {
    ClimbIO io;
    ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public ClimbSubsystem(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Climb", inputs);
        Logger.recordOutput("Superstructure/Climbing", inputs.climbVoltage > 1.0);
        io.updateInputs(inputs);
    }

    public Command setSpeed(Supplier<Double> climbSpeed) {
        return this.run(() -> {
            io.setSpeed(climbSpeed.get());
        });
    }

    public Command setVoltage(Supplier<Double> climbVoltage) {
        return this.run(() -> {
            io.setVoltage(climbVoltage.get());
        });
    }
}