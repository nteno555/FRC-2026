package frc2025;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.commands.AutoAimCommands;
import frc2025.commands.DriveToPoseCommand;
import frc2025.commands.FollowPathCommand;
import frc2025.commands.SuperstructureCommands;
import frc2025.config.SubsystemConstants;
import frc2025.subsystems.drive.DriveSubsystem;
import frc2025.util.Alliance;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.List;
import java.util.function.Supplier;

public class Autos {
    private DriveSubsystem drive;

    private RobotContainer robotContainer;

    public Autos(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drive = robotContainer.getDrive();

        System.out.println(this.robotContainer);
    }

    /* Path Commands */


    private Command intakePath(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }

        PathPlannerPath finalPath = path;
        if (Robot.isSimulation()) {
            return new FollowPathCommand(drive, () -> finalPath, () -> true, true);
            //.raceWith(robotContainer.getIntake().setIntaking(() -> true));
            //.andThen(new WaitCommand(0.2));
        }

        return new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true);
        //.raceWith(robotContainer.getIntake().setIntaking(() -> true));
        //.andThen(new WaitCommand(0.2));
    }




    private Pose2d mirrorPose(Pose2d pose) {
        Translation2d translation = pose.getTranslation();
        return new Pose2d(translation.getX(), FlippingUtil.fieldSizeY - translation.getY(), pose.getRotation().times(-1));
    }

    private Supplier<Pose2d> startPoseSupplier(Pose2d pose) {
        return () -> {
            if (Alliance.get()) {
                return FlippingUtil.flipFieldPose(pose);
            }
            return pose;
        };
    }

    public Command resetOdometryIntakePath(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            System.out.println("BAD PATH BAD PATH BAD PATH");
            return null;
        }

        PathPlannerPath finalPath = path;
        if (Robot.isSimulation()) {
            return new FollowPathCommand(drive, () -> finalPath, () -> true, true);
            //.raceWith(robotContainer.getIntake().setIntaking(() -> true));
            //.andThen(new WaitCommand(0.2));
        }

        Command followPathCommand = new FollowPathCommand(drive, () -> finalPath, () -> Alliance.get(), true);
        Command resetOdom = this.drive.resetOdometryCommand(() -> finalPath.getStartingDifferentialPose());
        Command intakePathFinalCommand = resetOdom.andThen(followPathCommand);
        return intakePathFinalCommand;
        //.raceWith(robotContainer.getIntake().setIntaking(() -> true));
        //.andThen(new WaitCommand(0.2));
    }

    
    public Command Taxi() {
        Command auto = drive.driveRobotRelative(() -> new ChassisSpeeds(1, 0, 0))
                .withTimeout(2.0)
                .andThen(drive.driveRobotRelative(() -> new ChassisSpeeds(0, 0, 0)));
        return auto;
    }

    public Command testAuto() {
        return resetOdometryIntakePath("TestPath");
    }
    
}