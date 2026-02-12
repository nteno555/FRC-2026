package frc2025.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc2025.RobotContainer;

import java.util.function.Supplier;

public class SuperstructureCommands {
    private static boolean hoodNear(RobotContainer robotContainer, double position) {
        return Math.abs(robotContainer.getHood().getPosition() - position) < 0.05;
    }

    public static Command hoodSetpointCommand(RobotContainer robotContainer, double position) {
        return robotContainer.getHood().setPosition(() -> position).until(() -> hoodNear(robotContainer, position));
    }

    public static Command intakePieceCommand(RobotContainer robotContainer) {
        return robotContainer.getIntake().setRollerVoltage(() -> 6.0);
    }

    public static Command shootCommand(RobotContainer robotContainer) {
        return robotContainer.getIndexer().setVoltage(() -> -3.0, () -> -3.0);
    }

    public static Command resetOdometry(RobotContainer robotContainer, Supplier<Pose2d> pose) {
        return robotContainer.getDrive().resetOdometryCommand(pose)
                .alongWith(new InstantCommand(() -> robotContainer.getVision().resetOdometry()));
    }
}
