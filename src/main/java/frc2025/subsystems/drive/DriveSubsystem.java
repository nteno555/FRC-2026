package frc2025.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2025.RobotState;
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

    public DriveSubsystem(DriveIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        //io.setControl(request.withSpeeds(applyAccelLimits()));
        io.setControl(request.withSpeeds(wantedSpeeds));
        RobotState.getInstance().addDriveData(inputs.odometryTimestamp, inputs.odometryTwist);
        /*RobotState.getInstance().setSwervePose(inputs.swervePoseMeters);*/

        //if(RobotState.getInstance().getHasNewVisionUpdate()) {
        if(RobotState.getInstance().getVisionUpdates().size() > 0) {
//            Logger.recordOutput("Drive/fuseVisionHeartbeat", Math.random());
            io.addVisionMeasurement(RobotState.getInstance().getVisionUpdates());
            RobotState.getInstance().clearVisionUpdates();
        }

        //Pose2d currentPose = io.samplePoseAt(Utils.getCurrentTimeSeconds()).get();
        //RobotState.getInstance().setVisionPoseBuiltin(currentPose);

        //RobotState.getInstance().setDriveAcceleration(io.)

        double speed = Math.sqrt(Math.pow(inputs.currentChassisSpeeds.vxMetersPerSecond, 2) + Math.pow(inputs.currentChassisSpeeds.vyMetersPerSecond, 2));
        RobotState.getInstance().setSpeed(speed);
        RobotState.getInstance().setRobotRelativeSpeeds(inputs.currentChassisSpeeds);
        // RobotState.getInstance().updateShootingWhileMovingVirtualPose();
        // RobotState.getInstance().updateShotVectorWhileMoving();
//        Logger.recordOutput("Drive/shotAutoIndexReady", getShotReady());
        RobotState.getInstance().setDriveShotReady(getShotReady());
        Logger.recordOutput("Drive/wantedSpeeds", wantedSpeeds);
        Logger.processInputs("RealOutputs/Drive", inputs);
        Logger.recordOutput("Vision/autoAimAngleError", Math.toDegrees(RobotState.getInstance().getVisionPose().getRotation().getRadians() - RobotState.getInstance().getAngleToVirtualTarget()));
//        Logger.recordOutput("Vision/targetVirtualPose", RobotState.getInstance().getTargetPose());
    }

    public void applyChassisSpeeds(ChassisSpeeds speeds) {
        wantedSpeeds = speeds;
        //io.setControl(request.withSpeeds(speeds));
    }

    public Command applyChassisSpeedsCommand(Supplier<ChassisSpeeds> speed) {
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
            if(!Alliance.get()) translation = translation.times(-1);
            return new ChassisSpeeds(translation.getX(), translation.getY(), omega.getAsDouble());
        });
    }

    public Command driveRobotRelative(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        return this.applyChassisSpeedsCommand(() -> chassisSpeedsSupplier.get());
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

    public ChassisSpeeds getRobotRelativeSpeeds() { return inputs.currentChassisSpeeds; }
    public ChassisSpeeds getFieldRelativeSpeeds() { return ChassisSpeeds.fromRobotRelativeSpeeds(inputs.currentChassisSpeeds, inputs.swervePoseMeters.getRotation()); }

    private ChassisSpeeds applyAccelLimits() {
        double dt = Timer.getFPGATimestamp() - prevTimestamp;

        ChassisSpeeds currentSpeeds = getRobotRelativeSpeeds();
        ChassisSpeeds wantedAccel = wantedSpeeds.minus(currentSpeeds).div(dt);

        Logger.recordOutput("Drive/wantedAccel", wantedAccel);
        // tilt limits
        wantedAccel.vxMetersPerSecond = MathUtil.clamp(wantedAccel.vxMetersPerSecond, -FORWARD_ACCEL_MAX, FORWARD_ACCEL_MAX);
        wantedAccel.vyMetersPerSecond = MathUtil.clamp(wantedAccel.vyMetersPerSecond, -SIDE_ACCEL_MAX, SIDE_ACCEL_MAX);


        prevTimestamp = Timer.getFPGATimestamp();

        return currentSpeeds.plus(wantedAccel.times(dt));
    }

    public ChassisSpeeds getTeleopSpeeds(double x, double y, double omega) {
        Translation2d translation = new Translation2d(x, y);
        if(!Alliance.get()) translation = translation.times(-1);
        return new ChassisSpeeds(translation.getX(), translation.getY(), omega);
    }

    public boolean getShotReady() {
        double angleToVirtualTarget = RobotState.getInstance().getAngleToVirtualTarget();
        double robotAngle = RobotState.getInstance().getVisionPose().getRotation().getRadians();
        return Math.abs((angleToVirtualTarget - robotAngle) % (Math.PI * 2)) < Math.toRadians(15.0);
    }

    // characterization

    private SwerveSysidVoltageRequest driveVoltageRequest = new SwerveSysidVoltageRequest(true);

	private SysIdRoutine m_driveSysIdRoutine =
			new SysIdRoutine(
					new SysIdRoutine.Config(),
					new SysIdRoutine.Mechanism(
							(Voltage volts) -> this.io.setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
							null,
							this));

	private SwerveSysidVoltageRequest steerVoltageRequest = new SwerveSysidVoltageRequest(false);

	private SysIdRoutine m_steerSysIdRoutine =
			new SysIdRoutine(
					new SysIdRoutine.Config(),
					new SysIdRoutine.Mechanism(
							(Voltage volts) -> this.io.setControl(steerVoltageRequest.withVoltage(volts.in(Volts))),
							null,
							this));

	private SysIdRoutine m_slipSysIdRoutine =
			new SysIdRoutine(
					new SysIdRoutine.Config(Volts.of(0.25).per(Seconds.of(1).unit()), null, null),
					new SysIdRoutine.Mechanism(
							(Voltage volts) -> this.io.setControl(driveVoltageRequest.withVoltage(volts.in(Volts))),
							null,
							this));

	public Command runDriveQuasiTest(SysIdRoutine.Direction direction) {
		return m_driveSysIdRoutine.quasistatic(direction);
	}

	public Command runDriveDynamicTest(SysIdRoutine.Direction direction) {
		return m_driveSysIdRoutine.dynamic(direction);
	}

	public Command runSteerQuasiTest(SysIdRoutine.Direction direction) {
		return m_steerSysIdRoutine.quasistatic(direction);
	}

	public Command runSteerDynamicTest(SysIdRoutine.Direction direction) {
		return m_steerSysIdRoutine.dynamic(direction);
	}

	public Command runDriveSlipTest() {
		return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
	}

    public Translation2d getVisionVelocity() {
        double currentTime = Utils.getCurrentTimeSeconds();
        Pose2d currentPose = io.samplePoseAt(currentTime).get();
        Pose2d previousPose = io.samplePoseAt(currentTime - 0.02).get();
        return currentPose.minus(previousPose).div(0.02).getTranslation();
    }
}
