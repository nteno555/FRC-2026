package frc2025.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc2025.RobotState;
import frc2025.config.ShooterTreeMap;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    double wantedVelocity = 0.0;
    double wantedVoltage = 0.0;
    boolean usingVoltage = false;
    ShooterIO io;
    ShooterIOInputsAutoLogged inputs;
    double rampVoltage = 0.0;
    double prevVelocity = 0.0;
    double prevTimestamp = -1;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        Logger.processInputs("RealOutputs/Shooter", inputs);
        Logger.recordOutput("Superstructure/masterVelocityRPS", inputs.masterVelocityRPS);
        Logger.recordOutput("Shooter/wantedVelocity", wantedVelocity);
        Logger.recordOutput("Shooter/shotReadyAF", getShotReadyAF());
        io.updateInputs(this.inputs);
    }
    public double[] getInputs(){
        return new double[]{wantedVelocity,inputs.masterVelocityRPS};
    }
    public Command characterizeStaticVoltage() {
        return this.runOnce(() -> rampVoltage = 0.0)
            .andThen(this.run(() -> {
                rampVoltage += 0.02;
                io.setVoltage(rampVoltage);
                Logger.recordOutput("Shooter/staticRampVoltage", rampVoltage);
                Logger.recordOutput("Shooter/staticVelocityRPS", inputs.masterVelocityRPS);
            }).until(() -> (Math.abs(inputs.masterVelocityRPS) > 0.5)))
            .finallyDo((interrupted) -> {
                Logger.recordOutput("Shooter/measuredkSVolts", rampVoltage);
                io.setVoltage(0.0);
            });
    }

    public Command characterizeVoltageStep(Supplier<Double> voltage) {
        return this.runEnd(() -> {
            io.setVoltage(voltage.get());
            Logger.recordOutput("Shooter/currentStepVoltage", voltage.get());
            Logger.recordOutput("Shooter/currentStepVelocityRPS", inputs.masterVelocityRPS);
        }, () -> io.setVoltage(0.0));
    }

    public Command dynamicVoltageStep(Supplier<Double> voltage) {
        return this.runEnd(() -> {
            double currentTimeStamp = Timer.getFPGATimestamp();
            io.setVoltage(voltage.get());
            double acceleration = 0.0;
            if (this.prevTimestamp == -1) {
                double deltaV = inputs.masterVelocityRPS - this.prevVelocity;
                double dt = currentTimeStamp - prevTimestamp;
                acceleration = deltaV/dt;
            }
            Logger.recordOutput("Shooter/acceleration", acceleration);
            this.prevTimestamp = currentTimeStamp;
            this.prevVelocity = inputs.masterVelocityRPS;
        }, () -> io.setVoltage(0.0));
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return this.run(() -> {
            usingVoltage = true;
            wantedVoltage = voltage.get();
            io.setVoltage(voltage.get());
        });
    }

    public Command setSpeed(Supplier<Double> speed) {
        return this.runEnd(() -> {
            usingVoltage = false;
            wantedVelocity = speed.get();
            io.setSpeed(speed.get(), 0.0);
        }, () -> {wantedVelocity = 0.0; io.setSpeed(0.0, 0.0);});
    }


    public Command setSpeed(Supplier<Double> speed, Supplier<Double> acceleration) {
        return this.runEnd(() -> {
            usingVoltage = false;
            wantedVelocity = speed.get();
            io.setSpeed(speed.get(), acceleration.get());
        }, () -> wantedVelocity = 0.0);
    }

    public Command setSpeedWithShuttling() {
        // if (RobotState.getShuttling()) {
        //     return setSpeed(()->ShooterTreeMap.shuttlingRPSTreeMap.getInterpolated(RobotState.getDistanceToHub()));
        // } else {
        //     return setSpeed(()->ShooterTreeMap.lowerWheelTreeMap.getInterpolated(RobotState.getDistanceToHub()));
        // }
        return setSpeed(()->ShooterTreeMap.lowerWheelTreeMap.getInterpolated(RobotState.getDistanceToHub()));
//        return setSpeed(() -> {
//            Pose2d robotPose = RobotState.getDelayedShooterPose();
//            Pose2d targetPose = RobotState.getShootingWhileMovingVirtualPose();
//            double dist = robotPose.getTranslation().getDistance(targetPose.getTranslation());
//
//            if (RobotState.getShuttling()) {
//                return ShooterTreeMap.shuttlingRPSTreeMap.getInterpolated(dist);
//            }
//
//            return ShooterTreeMap.lowerWheelTreeMap.getInterpolated(dist);
//        });
    }


    public boolean getShotReadyAF() {
        return Math.abs(wantedVelocity - inputs.masterVelocityRPS) < 1.2 &&
        Math.abs(wantedVelocity) > 1.0;
    }

    public double getCurrent() {
        return inputs.totalCurrent;
    }

    public double getVelocity() {
        return inputs.masterVelocityRPS;
    }
}