// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommand extends SequentialCommandGroup {
	/** Creates a new AutoCommand. */
	public AutoCommand(boolean doShoot, double startAngle, double startDelay, DriveSubsystem robotDrive,
			ShooterSubsystem shooter) {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
				AutoConstants.kMaxSpeedMetersPerSecond,
				AutoConstants.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(DriveConstants.kDriveKinematics);

		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(startAngle))),
				List.of(),
				// End 4 meters straight ahead of where we started, facing forward
				new Pose2d(4, 0, new Rotation2d(0)),
				config);

		var thetaController = new ProfiledPIDController(
				AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
				exampleTrajectory,
				robotDrive::getPose, // Functional interface to feed supplier
				DriveConstants.kDriveKinematics,

				// Position controllers
				new PIDController(AutoConstants.kPXController, 0, 0),
				new PIDController(AutoConstants.kPYController, 0, 0),
				thetaController,
				robotDrive::setModuleStates,
				robotDrive);

		// Reset odometry to the initial pose of the trajectory, run path following
		// command, then stop at the end.
		addCommands(
				Commands.runOnce(
						() -> robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
				Commands.waitSeconds(startDelay),
				Commands.either(
						shooter.shootCommand(0.4), Commands.none(),
						() -> doShoot),
				swerveControllerCommand,
				Commands.runOnce(robotDrive::stop));
	}
}
