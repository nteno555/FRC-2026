package frc2025.subsystems;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.arm.ArmSubsystem;
import frc2025.subsystems.elevator.ElevatorSubsystem;
import frc2025.subsystems.endEffector.EndEffectorSubsystem;
import frc2025.subsystems.funnel.FunnelSubsystem;
import frc2025.subsystems.climber.ClimberSubsystem;
import org.jgrapht.Graph;
import org.jgrapht.alg.shortestpath.BFSShortestPath;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class Superstructure extends SubsystemBase {
    public enum State {
        IDLE,
        EJECT_CORAL,
        EJECT_L1,
        L4_CORAL_PRE,
        L3_CORAL_PRE,
        L2_CORAL_PRE,
        L3_ALGAE_INTAKE,
        L2_ALGAE_INTAKE,
        STOW,
        STOW_WITH_ALGAE,
        HANDOFF,
        CLIMB,
        DECLIMB,
        NET_SCORE_PRE,
        NET_SCORE_EJECT,
        PROCESSOR_SCORE_PRE,
        PROCESSOR_SCORE_EJECT,
    }

    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;
    FunnelSubsystem funnelSubsystem;
    EndEffectorSubsystem endEffectorSubsystem;
    ClimberSubsystem climberSubsystem;


    public LoggedMechanism2d elevatorArmMech = new LoggedMechanism2d(3, 3);
	public LoggedMechanismRoot2d root = elevatorArmMech.getRoot("root", 1.5, 0);
	public LoggedMechanismLigament2d elevatorMech = root.append(new LoggedMechanismLigament2d("elevator", 1, 90));
	public LoggedMechanismLigament2d armMech = elevatorMech.append(new LoggedMechanismLigament2d("arm", 0.5, 90));


    private Graph<State, DefaultEdge> graph = new DefaultDirectedGraph<>(DefaultEdge.class);
    private BFSShortestPath<State, DefaultEdge> shortestPathGenerator;

    private boolean reachedState = false;
    private int currentStateIndex;
    private State currentState;
    private List<State> pathToWanted;

    private ArrayList<Trigger> edgeTriggers = new ArrayList<>();

    private double elevatorSetpoint, armSetpoint;
    private boolean forceStow;

    public Superstructure(
            ElevatorSubsystem elevator,
            ArmSubsystem arm,
            FunnelSubsystem funnel,
            EndEffectorSubsystem endEffector,
            ClimberSubsystem climber
    ) {
        elevatorSubsystem = elevator;
        armSubsystem = arm;
        funnelSubsystem = funnel;
        endEffectorSubsystem = endEffector;
        climberSubsystem = climber;

        reset();

        elevatorSetpoint = 0;
        armSetpoint = 0;

        for(var state: State.values()) {
            graph.addVertex(state);
            //addEdge(state, State.IDLE, new InstantCommand());
        }

        Trigger forceStowTrigger = new Trigger(() -> forceStow)
                .onTrue(safeStowCommand(0, 0));

        addEdge(State.IDLE, State.STOW, safeStowCommand(2, 0));
        addEdge(State.L4_CORAL_PRE, State.EJECT_CORAL, endEffectorSubsystem.setVoltage(() -> -5.0));
        addEdge(State.L3_CORAL_PRE, State.EJECT_CORAL, endEffectorSubsystem.setVoltage(() -> -5.0));
        addEdge(State.L2_CORAL_PRE, State.EJECT_CORAL, endEffectorSubsystem.setVoltage(() -> -5.0));
        addEdge(State.STOW, State.L4_CORAL_PRE, safeElevatorArmCommand(SubsystemConstants.elevatorL4Pos, SubsystemConstants.armL4Pos).andThen(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.STOW, State.L3_CORAL_PRE, safeElevatorArmCommand(SubsystemConstants.elevatorL3Pos, SubsystemConstants.armL3Pos).andThen(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.STOW, State.L2_CORAL_PRE, safeElevatorArmCommand(SubsystemConstants.elevatorL2Pos, SubsystemConstants.armL2Pos).andThen(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.HANDOFF, State.L4_CORAL_PRE, safeElevatorArmCommand(SubsystemConstants.elevatorL4Pos, SubsystemConstants.armL4Pos).andThen(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.HANDOFF, State.L3_CORAL_PRE, safeElevatorArmCommand(SubsystemConstants.elevatorL3Pos, SubsystemConstants.armL3Pos).andThen(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.HANDOFF, State.L2_CORAL_PRE, safeElevatorArmCommand(SubsystemConstants.elevatorL2Pos, SubsystemConstants.armL2Pos).andThen(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.HANDOFF, State.EJECT_L1, endEffectorSubsystem.setVoltage(() -> -9.0).alongWith((funnelSubsystem.setVoltage(() -> -9.0))));
        addEdge(State.HANDOFF, State.STOW,
                (safeElevatorArmCommand(0, 0)
                        .alongWith(endEffectorSubsystem.setVoltage(() -> 9.0)))
                        .alongWith(funnelSubsystem.setVoltage(() -> 0.0))
                        .andThen(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.STOW, State.HANDOFF,
                unsafeElevatorArmCommand(SubsystemConstants.elevatorHandoffPos, SubsystemConstants.armHandoffPos)
                        .alongWith(endEffectorSubsystem.setVoltage(() -> 5.0))
                        .andThen(funnelSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.EJECT_CORAL, State.STOW, safeStowCommand(0, 0));
        addEdge(State.EJECT_L1, State.STOW, safeStowCommand(0, 0));
        addEdge(State.L4_CORAL_PRE, State.STOW, safeStowCommand(2, 0));
        addEdge(State.L3_CORAL_PRE, State.STOW, safeStowCommand(2, 0));
        addEdge(State.L2_CORAL_PRE, State.STOW, safeStowCommand(2, 0));
        addEdge(State.STOW, State.L3_ALGAE_INTAKE, safeElevatorArmCommand(SubsystemConstants.elevatorL3AlgaePos, SubsystemConstants.armRemoveAlgaePos)
                .alongWith(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.STOW, State.L2_ALGAE_INTAKE, safeElevatorArmCommand(SubsystemConstants.elevatorL2AlgaePos, SubsystemConstants.armRemoveAlgaePos)
                .alongWith(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.L3_ALGAE_INTAKE, State.STOW_WITH_ALGAE, safeElevatorArmCommandSlow(0, SubsystemConstants.armRemoveAlgaePos)
                .alongWith(endEffectorSubsystem.setVoltage(() -> 6.0)));
        addEdge(State.L2_ALGAE_INTAKE, State.STOW_WITH_ALGAE, safeElevatorArmCommandSlow(0, SubsystemConstants.armRemoveAlgaePos)
                .alongWith(endEffectorSubsystem.setVoltage(() -> 6.0)));
        addEdge(State.STOW_WITH_ALGAE, State.STOW, safeElevatorArmCommand(0, 0)
                .alongWith(endEffectorSubsystem.setVoltage(() -> 0.0)));
        addEdge(State.STOW, State.STOW_WITH_ALGAE, safeElevatorArmCommand(SubsystemConstants.elevatorL2AlgaePos, SubsystemConstants.armRemoveAlgaePos)
                .alongWith(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.STOW_WITH_ALGAE, State.NET_SCORE_PRE, safeElevatorArmCommand(SubsystemConstants.elevatorBargePos, SubsystemConstants.armBargePos));
        addEdge(State.STOW_WITH_ALGAE, State.PROCESSOR_SCORE_PRE, safeElevatorArmCommandSlow(SubsystemConstants.elevatorProcessorPos, SubsystemConstants.armProcessorPos));
        addEdge(State.PROCESSOR_SCORE_PRE, State.STOW_WITH_ALGAE, safeElevatorArmCommandSlow(SubsystemConstants.elevatorL2AlgaePos, SubsystemConstants.armRemoveAlgaePos));
        addEdge(State.PROCESSOR_SCORE_PRE, State.PROCESSOR_SCORE_EJECT, safeElevatorArmCommandSlow(SubsystemConstants.elevatorProcessorPos, SubsystemConstants.armProcessorPos).alongWith(endEffectorSubsystem.setVoltage(() -> -5.0)));
        addEdge(State.PROCESSOR_SCORE_EJECT, State.PROCESSOR_SCORE_PRE, safeElevatorArmCommandSlow(SubsystemConstants.elevatorProcessorPos, SubsystemConstants.armProcessorPos).alongWith(endEffectorSubsystem.setVoltage(() -> 5.0)));
        addEdge(State.NET_SCORE_PRE, State.NET_SCORE_EJECT, safeElevatorArmCommand(SubsystemConstants.elevatorBargePos, SubsystemConstants.armBargePos).alongWith(endEffectorSubsystem.setVoltage(() -> -5.0)));
        addEdge(State.NET_SCORE_PRE, State.STOW_WITH_ALGAE,
                endEffectorSubsystem.setVoltage(() -> 5.0)
                        .andThen(armSubsystem.setPosition(() -> -25.0)).until(() -> armSubsystem.isNear(-25.0))
                        .andThen(elevatorSubsystem.setPosition(() -> 0.0)).until(() -> elevatorSubsystem.isNear(-0.0))
                .andThen(safeElevatorArmCommand(0, SubsystemConstants.armRemoveAlgaePos))
        );
        addEdge(State.NET_SCORE_EJECT, State.STOW_WITH_ALGAE,
                endEffectorSubsystem.setVoltage(() -> 0.0)
                        .andThen(armSubsystem.setPosition(() -> -25.0).until(() -> armSubsystem.isNear(-25.0)))
                        .andThen(elevatorSubsystem.setPosition(() -> 0.0)).until(() -> elevatorSubsystem.isNear(0))
                .andThen(safeElevatorArmCommand(0, SubsystemConstants.armRemoveAlgaePos))
        );
        addEdge(State.STOW, State.CLIMB, climber.setVoltage(() -> 5.0));
        addEdge(State.CLIMB, State.DECLIMB, climber.setVoltage(() -> -5.0));
        addEdge(State.CLIMB, State.STOW, new InstantCommand());
        addEdge(State.DECLIMB, State.STOW, new InstantCommand());
        addEdge(State.DECLIMB, State.CLIMB, climber.setVoltage(() -> 5.0));


        shortestPathGenerator = new BFSShortestPath<>(graph);

    }

    @Override
    public void periodic() {
        reachedState = false;

        currentState = pathToWanted.get(currentStateIndex);
        if(currentStateIndex >= pathToWanted.size()-1)
            reachedState = true;

        elevatorMech.setLength(1 + elevatorSubsystem.getPosition() * 1/20);
        armMech.setAngle(armSubsystem.getRealAngle());
        Logger.recordOutput("Superstructure/elevatorArmMech", elevatorArmMech);
        Logger.recordOutput("Superstructure/currentState", currentState);
        Logger.recordOutput("Superstructure/currentStateIndex", currentStateIndex);
        Logger.recordOutput("Superstructure/pathLength", pathToWanted.size());
        Logger.recordOutput("Superstructure/wantedState", pathToWanted.get(pathToWanted.size()-1));
        Logger.recordOutput("Superstructure/hasPiece", hasPiece());
        Logger.recordOutput("Superstructure/reachedWantedState", reachedState);
    }

    public void reset() {

        currentStateIndex = 0;
        currentState = State.STOW;
        pathToWanted = List.of(State.STOW);

        elevatorSetpoint = 0;
        armSetpoint = 0;
    }

    private boolean elevatorArmNear(double elevatorPos, double armPos) {
        //return Math.abs(elevatorSubsystem.getPosition() - elevatorPos) < 2.0 && Math.abs(armSubsystem.getPosition() - armPos) < 2.5;
        return elevatorSubsystem.isNear(elevatorPos) && armSubsystem.isNear(armPos);
    }

    private Trigger makeEdgeTrigger(State curState, State nxtState, Command command) {
        return new Trigger(() -> {
            if(currentStateIndex >= pathToWanted.size()-1) return false;
            return pathToWanted.get(currentStateIndex) == curState && pathToWanted.get(currentStateIndex+1) == nxtState;
        }).whileTrue(command.andThen(new InstantCommand(() -> currentStateIndex += 1)));
    }

    private void addEdge(State u, State v, Command edgeCommand) {
        graph.addEdge(u, v);
        edgeTriggers.add(makeEdgeTrigger(u, v, edgeCommand));
    }
    private Command unsafeElevatorArmCommand(double elevatorPosition, double armPosition) {
        /*return new InstantCommand(() -> {elevatorSetpoint = elevatorPosition; armSetpoint = armPosition; })
                .andThen(elevatorSubsystem.setPosition(() -> elevatorPosition))
                .alongWith(armSubsystem.setPosition(() -> armPosition))
                .until(() -> elevatorArmNear(elevatorPosition, armPosition));*/
        return elevatorSubsystem.setPosition(() -> elevatorPosition)
                .alongWith(armSubsystem.setPosition(() -> armPosition))
                .until(() -> elevatorArmNear(elevatorPosition, armPosition));
    }

    private Command safeStowCommand(double effectorVoltage, double funnelVoltage) {
        return (new InstantCommand(() -> {elevatorSetpoint = 0; armSetpoint = 0; }))
                .andThen(armSubsystem.setPosition(() -> 0.0)).until(() -> armSubsystem.getPosition() > -2.5)
                .andThen(elevatorSubsystem.setPosition(() -> 0.0))//.until(() -> elevatorSubsystem.getPosition() < 2.5))
        //return safeElevatorArmCommand(0, 0)
                        .alongWith(endEffectorSubsystem.setVoltage(() -> effectorVoltage))
                        .alongWith(funnelSubsystem.setVoltage(() -> funnelVoltage))
                .until(() -> elevatorArmNear(0, 0));
    }

    public Command safeElevatorArmCommand(double elevatorPosition, double armPosition) {
        if(armPosition > -30) {
            //return (new InstantCommand(() -> {elevatorSetpoint = elevatorPosition; armSetpoint = armPosition; })
             //       .andThen(armSubsystem.setPosition(() -> armPosition))
            return armSubsystem.setPosition(() -> armPosition)
                    .alongWith(
                            (new WaitUntilCommand(() -> armSubsystem.getPosition() > -30))
                                    .andThen(elevatorSubsystem.setPosition(() -> elevatorPosition)))
                    .until(() -> elevatorArmNear(elevatorPosition, armPosition));
        } else {
            return new ConditionalCommand(
                armSubsystem.setPosition(() -> -30.0).until(() -> armSubsystem.isNear(-30.0)),
                new InstantCommand(),
                () -> armSubsystem.getPosition() < -30.0
            )
                .andThen(elevatorSubsystem.setPosition(() -> elevatorPosition)).until(() -> elevatorSubsystem.isNear(elevatorPosition))
                .andThen(armSubsystem.setPosition(() -> armPosition))
                .until(() -> elevatorArmNear(elevatorPosition, armPosition));
        }
//        return elevatorSubsystem.setPosition(() -> elevatorPosition).until(() -> elevatorSubsystem.isAligned())
//                .andThen(armSubsystem.setPosition(() -> armPosition));
//        return new ConditionalCommand(
//                armSubsystem.setPosition(() -> -30.0).until(() -> armSubsystem.isAligned()),
//                new InstantCommand(),
//                () -> armSubsystem.getPosition() < -30.0
//        )
//                .andThen(elevatorSubsystem.setPosition(() -> elevatorPosition)).until(() -> elevatorSubsystem.isAligned())
//                .andThen(armSubsystem.setPosition(() -> armPosition))
//                .until(() -> armSubsystem.isAligned() && elevatorSubsystem.isAligned());

    }
    private Command safeElevatorArmCommandSlow(double elevatorPosition, double armPosition) {
        if(armPosition > -30) {
            return armSubsystem.setPosition(() -> armPosition)
                    .alongWith(
                            (new WaitUntilCommand(() -> armSubsystem.getPosition() > -30))
                                    .andThen(elevatorSubsystem.setPositionSlow(() -> elevatorPosition)))
                    .until(() -> elevatorArmNear(elevatorPosition, armPosition));
        } else {
            return new ConditionalCommand(
                    armSubsystem.setPosition(() -> -30.0).until(() -> armSubsystem.isNear(-30.0)),
                    new InstantCommand(),
                    () -> armSubsystem.getPosition() < -30.0
            )
                    .andThen(elevatorSubsystem.setPositionSlow(() -> elevatorPosition)).until(() -> elevatorSubsystem.isNear(elevatorPosition))
                    .andThen(armSubsystem.setPosition(() -> armPosition))
                    .until(() -> elevatorArmNear(elevatorPosition, armPosition));
        }
    }

    public Command setWantedStateCommand(State wantedState) { return setWantedStateCommand(() -> wantedState); }

    public Command setWantedStateCommand(Supplier<State> wantedState) {
        return new WaitUntilCommand(() -> reachedState)
                        .andThen(this.runOnce(() -> {
                            var graphPath = shortestPathGenerator.getPath(currentState, wantedState.get());
                            pathToWanted = graphPath.getVertexList();
                            currentStateIndex = 0;
                            currentState = pathToWanted.get(0);
                        }).alongWith(new WaitCommand(0.01).andThen(new WaitUntilCommand(() -> reachedState)))); // NEED TO FIX LOL
    }

    public boolean hasPiece() {
        return endEffectorSubsystem.getHasPiece();
    }

    public State getCurrentState() { return currentState; }
}
