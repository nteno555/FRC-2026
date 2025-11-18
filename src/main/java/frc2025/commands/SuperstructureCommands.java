package frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc2025.subsystems.indexer.IndexerSubsystem;
import frc2025.subsystems.intake.IntakeSubsystem;
import frc2025.subsystems.pivot.PivotSubsystem;

public class SuperstructureCommands {
    public static Command intakePieceCommand(IndexerSubsystem indexer, IntakeSubsystem intake, PivotSubsystem pivot) {
        return new InstantCommand();
    }
}
