package frc2025;

import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

import com.ctre.phoenix6.SignalLogger;
import com.esotericsoftware.minlog.Log;
import com.pathplanner.lib.commands.FollowPathCommand;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.json.JSONObject;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	RobotContainer robotContainer;

	public Robot() {
		Logger.addDataReceiver(new WPILOGWriter());
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();
		robotContainer = new RobotContainer();

	}
	@Override
	public void robotInit() {
		RobotController.setBrownoutVoltage(5.5);
        System.out.println("Yay you deployed");
		FollowPathCommand.warmupCommand().schedule();
        SignalLogger.setPath("/home/lvuser/logs/");
        SignalLogger.start();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		robotContainer.periodic();
	}

	@Override
	public void autonomousInit() {
		robotContainer.autoInit();
	}

	@Override
	public void teleopInit() {
		robotContainer.teleopInit();
	}
}
