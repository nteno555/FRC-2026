package frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc2025.RobotContainer;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.indexer.IndexerSubsystem;
import frc2025.subsystems.intake.IntakeSubsystem;
import frc2025.subsystems.pivot.PivotSubsystem;

public class SuperstructureCommands {
    private static boolean pivotNear(RobotContainer robotContainer, double position) {
        return Math.abs(robotContainer.getPivot().getPosition() - position) < 0.05;
    }

    public static Command pivotSetpointCommand(RobotContainer robotContainer, double position) {
        return robotContainer.getPivot().setPosition(() -> position).until(() -> pivotNear(robotContainer, position));
    }

    public static Command intakePieceCommand(RobotContainer robotContainer) {
        return robotContainer.getIntake().setVoltage(() -> 6.0);
    }

    public static Command shootCommand(RobotContainer robotContainer) {
        return robotContainer.getIndexer().setVoltage(() -> -3.0, () -> -3.0);
    }
}
