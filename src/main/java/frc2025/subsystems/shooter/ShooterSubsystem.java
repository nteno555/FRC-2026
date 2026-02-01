package frc2025.subsystems.shooter;

import java.util.function.Supplier;

import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    double wantedVelocity = 0.0;
    double wantedVoltage = 0.0;
    boolean usingVoltage = false;
    double wantedPercentOutput = 0.0;
    ShooterIO io;
    ShooterIOInputsAutoLogged inputs;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Shooter", inputs);
        Logger.recordOutput("Shooter/shotReady", getShotReady());
        io.updateInputs(this.inputs);
        RobotState.getInstance().setShooterReady(getShotReady());
//        RobotState.getInstance().setShooterExitVelocity(getBallExitVelocity());
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            Logger.recordOutput("Shooter/wantedVoltage", voltage.get());
            usingVoltage = true;
            wantedVoltage = voltage.get();
            io.setVoltage(voltage.get());
        });
    }

    public Command setSpeed(Supplier<Double> speed) {
        return this.runEnd(() -> {
            Logger.recordOutput("Shooter/wantedSpeed", speed.get());
            usingVoltage = false;
            wantedVelocity = speed.get();
            io.setSpeed(speed.get());
        }, () -> {
            usingVoltage = false;
            io.setSpeed(0.0);
        });
    }

    public Command setTargetExitVelocity(Supplier<Double> exitVelocity) {
        return setSpeed(() -> solveExitVelocity(exitVelocity.get()));
    }

    public ShooterIO getIO() {
        return io;
    }

    public Command setPercentOutput(Supplier<Double> percentOutput) {
        return this.run( () -> {
            Logger.recordOutput("Shooter/wantedPercentOutput", percentOutput.get());
            usingVoltage = false;
            wantedPercentOutput = percentOutput.get();
            io.setPercentOutput(percentOutput.get());
            }
        );
    }

    public double getUpperVelocity() {
        return inputs.masterVelocityRPM;
    }
    public double getUpperWantedVelocity() {
        return wantedVelocity;
    }
    public boolean getShotReady() {
        return Math.abs(wantedVelocity - inputs.masterVelocityRPM) < 5.0 && wantedVelocity != 0.0;
    }

    public double getBallExitVelocity() {
        return toBallExitVelocity(inputs.masterVelocityRPM);
    }

    public double solveExitVelocity(double wantedExitVelocity) {
        // (top + bottom) / 2 * efficiency = wantedExitVelocity


        Logger.recordOutput("Shooter/wantedExitVelocity", wantedExitVelocity);
        double res = -wantedExitVelocity*2.0 / (SubsystemConstants.flywheelEfficiency) /
               (SubsystemConstants.topFlywheelGearRatio * SubsystemConstants.topFlywheelDiameter * Math.PI +
                SubsystemConstants.bottomFlywheelGearRatio * SubsystemConstants.bottomFlywheelDiameter * Math.PI);
        Logger.recordOutput("Shooter/solvedRps", res);
        return res;
    }

    public static double toBallExitVelocity(double wheelRPM) {
        double topVelocity = SubsystemConstants.topFlywheelGearRatio * wheelRPM * SubsystemConstants.topFlywheelDiameter * Math.PI;
        double bottomVelocity = SubsystemConstants.bottomFlywheelGearRatio * wheelRPM * SubsystemConstants.bottomFlywheelDiameter * Math.PI;
        return -(topVelocity + bottomVelocity) / 2.0 * SubsystemConstants.flywheelEfficiency;
    }
}