package frc2025.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2025.RobotContainer;

import java.util.function.Supplier;

public class SuperstructureCommands {
    public static Command resetOdometry(RobotContainer robotContainer, Supplier<Pose2d> pose) {
        return robotContainer.getVision().resetOdometryCommand().alongWith(
                robotContainer.getDrive().resetOdometryCommand(pose)
        );
    }
}
