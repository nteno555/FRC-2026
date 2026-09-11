package frc2025.subsystems.drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.commands.FollowPathCommand;
import frc2025.config.FieldConstants;
import frc2025.config.SubsystemConstants;
import frc2025.util.Alliance;
import frc2025.util.SwerveSysidVoltageRequest;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
//import static frc2025.subsystems.drive.DriveConstants.kMaxAngularVelocity;
//import static frc2025.subsystems.drive.DriveConstants.kSpeedAt12VoltsMps;

public class DriveSubsystem extends SubsystemBase {
    public static final double FORWARD_ACCEL_MAX = 30.0; // with all mechs down
    public static final double SIDE_ACCEL_MAX = 30.0; // with all mechs down
    public static final double SKID_ACCEL_MAX = 5.0;

    public static boolean slowmode = false;

    DriveIO io;
    DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private ChassisSpeeds wantedSpeeds = new ChassisSpeeds(0, 0, 0);
    private final SwerveRequest.ApplyRobotSpeeds request = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position);
    private double prevTimestamp = 0;
    private boolean sysIdActive = false;

    public DriveSubsystem(DriveIO io) {
        this.io = io;

        AutoBuilder.configure(
                () -> RobotState.getInstance().getVisionPose(), // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds, feedforwards), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards for things like gearbox friction and motor back-EMF compensation
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(SubsystemConstants.translationkP, SubsystemConstants.translationkI, SubsystemConstants.translationkD), // Translation PID constants
                        new PIDConstants(SubsystemConstants.rotationkP, SubsystemConstants.rotationkI, SubsystemConstants.rotationkD) // Rotation PID constants
                ),
                getPPConfig(), // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
        io.addVisionMeasurement(RobotState.getInstance().getVisionUpdates());
        io.updateInputs(inputs);
        if (!sysIdActive) {
            io.setControl(request.withSpeeds(wantedSpeeds));
        }

        RobotState.getInstance().setPose(inputs.swervePoseMeters);
        RobotState.getInstance().setRobotRelativeSpeeds(inputs.currentChassisSpeeds);
        RobotState.getInstance().setPigeonAcceleration(inputs.pigeonAcceleration);
        Logger.recordOutput("Drive/wantedSpeeds", wantedSpeeds);
        Logger.processInputs("RealOutputs/Drive", inputs);
    }

    private double modAngle(double angle) {
        angle = angle % (2*Math.PI);
        if(angle > Math.PI) return angle - 2*Math.PI;
        if(angle < -Math.PI) return angle + 2*Math.PI;
        return angle;
    }

    public Command driveAutoAim() {
        Supplier<Double> omegaSpeed = () -> {
            Pose2d virtualTarget = FieldConstants.blueNetPos;
            if (virtualTarget == null) {
                return 0.0;
            }
            
            Pose2d shooterPose = RobotState.getInstance().getVisionPose();

            double desiredHeading = Math.atan2(
                    virtualTarget.getY() - shooterPose.getY(),
                    virtualTarget.getX() - shooterPose.getX()
            );

            double currentHeading = RobotState.getInstance().getVisionPose().getRotation().getRadians();
            double angleError = modAngle(desiredHeading - currentHeading);

            Logger.recordOutput("Drive/angleError", angleError);

            double feedback = SubsystemConstants.alignRotationkP * angleError;
            double feedforward = RobotState.getInstance().getVirtualTargetAngularVelocity();

            double omega = feedforward + feedback;
            return MathUtil.clamp(omega, -SubsystemConstants.autoAimMaxAngularVelocity, SubsystemConstants.autoAimMaxAngularVelocity);
        };

        Logger.recordOutput("Drive/Omega", omegaSpeed.get());
    

        return applyChassisSpeedsCommand(() -> new ChassisSpeeds(0, 0, -omegaSpeed.get()));
    }

    public boolean getShotReady() {
        Pose2d shooterPose = RobotState.getInstance().getDelayedShooterPose();
        Pose2d virtualTarget = RobotState.getInstance().getShootingWhileMovingVirtualPose();
        double desiredHeading = Math.atan2(
                virtualTarget.getY() - shooterPose.getY(),
                virtualTarget.getX() - shooterPose.getX()
        );

        double currentHeading = RobotState.getInstance().getVisionPose().getRotation().getRadians();
        double angleError = modAngle(desiredHeading - currentHeading);
        return (Math.abs(angleError) < 3);
    }



    public void applyChassisSpeeds(ChassisSpeeds speeds) {
        wantedSpeeds = speeds;
    }

    public Command applyChassisSpeedsCommand(Supplier<ChassisSpeeds> speed) {
        Logger.recordOutput("Drive/Speed", true);
        return this.run(() -> applyChassisSpeeds(speed.get()));
    }

    public Command driveFieldRelative(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        return this.applyChassisSpeedsCommand(() -> {
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSpeedsSupplier.get(),
                    RobotState.getInstance().getVisionPose().getRotation());
        });
    }

    public Command driveFieldRelativeVision(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        return this.applyChassisSpeedsCommand(() -> {
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSpeedsSupplier.get(),
                    RobotState.getInstance().getVisionPose().getRotation());
        });
    }

    public Command driveTeleop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
        return this.driveFieldRelative(() -> {
            Translation2d translation = new Translation2d(x.getAsDouble(), y.getAsDouble());
            if (!Alliance.get())
                translation = translation.times(-1);

            Logger.recordOutput("Drive/teleopWantedVelocity", translation.getNorm());

            return new ChassisSpeeds(translation.getX(), translation.getY(), omega.getAsDouble());
        });
    }

    public Command driveRobotRelative(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        return this.applyChassisSpeedsCommand(() -> chassisSpeedsSupplier.get());
    }

    public void drive(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        applyChassisSpeeds(speeds);
    }

    public Command resetOdometryCommand(Supplier<Pose2d> position) {
        return this.runOnce(() -> {
            resetOdometry(position.get());
        });
    }

    public SwerveModule[] getSwerveModules() {
        return io.getModules();
    }

    public void resetOdometry(Pose2d position) {
        io.resetOdometry(position);
        RobotState.getInstance().rezero(position);
    }

    public static RobotConfig getPPConfig() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            config = null;
        }
        return config;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return inputs.currentChassisSpeeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(inputs.currentChassisSpeeds,
                inputs.swervePoseMeters.getRotation());
    }

    private ChassisSpeeds applyAccelLimits() {
        double dt = Timer.getFPGATimestamp() - prevTimestamp;

        ChassisSpeeds currentSpeeds = getRobotRelativeSpeeds();
        ChassisSpeeds wantedAccel = wantedSpeeds.minus(currentSpeeds).div(dt);

        Logger.recordOutput("Drive/wantedAccel", wantedAccel);
        // tilt limits
        wantedAccel.vxMetersPerSecond = MathUtil.clamp(wantedAccel.vxMetersPerSecond, -FORWARD_ACCEL_MAX,
                FORWARD_ACCEL_MAX);
        wantedAccel.vyMetersPerSecond = MathUtil.clamp(wantedAccel.vyMetersPerSecond, -SIDE_ACCEL_MAX, SIDE_ACCEL_MAX);

        prevTimestamp = Timer.getFPGATimestamp();

        return currentSpeeds.plus(wantedAccel.times(dt));
    }

    public ChassisSpeeds getTeleopSpeeds(double x, double y, double omega) {
        Translation2d translation = new Translation2d(x, y);
        if (!Alliance.get())
            translation = translation.times(-1);
        return new ChassisSpeeds(translation.getX(), translation.getY(), omega);
    }

    public Command followPathCommand(PathPlannerPath path) {
        //return AutoBuilder.followPath(path);
        return new FollowPathCommand(this, ()-> path, ()-> Alliance.get(), false);
    }

    public Command followPathCommand(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            return followPathCommand(path);
        } catch (Exception e) {
            DriverStation.reportError("Could not load path: " + pathName, e.getStackTrace());
            return Commands.none();
        }
    }

    public Command followChoreoPathCommand(String pathName) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
            return followPathCommand(path);
        } catch (Exception e) {
            DriverStation.reportError("Could not load choreo path: " + pathName, e.getStackTrace());
            return Commands.none();
        }
    }

    public Command followPathBuiltinCommand(String pathName) {
        return followPathCommand(pathName);
    }


    // characterization

    private SwerveSysidVoltageRequest driveVoltageRequest = new SwerveSysidVoltageRequest(true);

    private SysIdRoutine m_driveSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> {
                        SignalLogger.start();
                        SignalLogger.writeString("drive-state", state.toString());
                    }),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> this.io.setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
                    null,
                    this));

    private SwerveSysidVoltageRequest steerVoltageRequest = new SwerveSysidVoltageRequest(false);

    private SysIdRoutine m_steerSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    Volts.of(7),
                    null,
                    (state) -> {
                        SignalLogger.start();
                        SignalLogger.writeString("steer-state", state.toString());
                    }),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> this.io.setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
                    null,
                    this));

    private SysIdRoutine m_slipSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1).unit()), null, null),
            new SysIdRoutine.Mechanism(
                    (Voltage volts) -> this.io.setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
                    null,
                    this));

    public Command runDriveQuasiTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.quasistatic(direction)
                .beforeStarting(() -> {
                    sysIdActive = true;
                })
                .finallyDo(() -> {
                    sysIdActive = false;
                });
    }

    public Command runDriveDynamicTest(SysIdRoutine.Direction direction) {
        return m_driveSysIdRoutine.dynamic(direction)
                .beforeStarting(() -> {
                    sysIdActive = true;
                })
                .finallyDo(() -> {
                    sysIdActive = false;
                });
    }

    public Command runSteerQuasiTest(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.quasistatic(direction)
                .beforeStarting(() -> {
                    sysIdActive = true;
                })
                .finallyDo(() -> {
                    sysIdActive = false;
                });
    }

    public Command runSteerDynamicTest(SysIdRoutine.Direction direction) {
        return m_steerSysIdRoutine.dynamic(direction)
                .beforeStarting(() -> {
                    sysIdActive = true;
                })
                .andThen(() -> {
                    sysIdActive = false;
                });
    }

    public Command runDriveSlipTest() {
        return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

}