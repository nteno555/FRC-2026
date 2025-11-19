package frc2025.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.config.SubsystemConstants;
import org.jgrapht.Graph;
import org.jgrapht.alg.shortestpath.BFSShortestPath;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import frc2025.subsystems.pivot.PivotSubsystem;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Superstructure {
    PivotSubsystem pivot;
    DriveSubsystem drive;
    VisionSubsystem vision;

    public Superstructure(PivotSubsystem pivot, DriveSubsystem drive, VisionSubsystem vision) {
        this.pivot = pivot;
        this.drive = drive;
        this.vision = vision;
    }

    // public Command intakeCommand() {
    //     return pivot.setPosition(() -> SubsystemConstants.intakeAngle).until(() -> pivot.isNear(SubsystemConstants.intakeAngle));
    // }
}