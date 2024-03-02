// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final ShooterSubsystem m_shooter = new ShooterSubsystem();
        private final ClimberSubsystem m_climber = new ClimberSubsystem();

        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        Joystick m_operatorsStick = new Joystick(OIConstants.kOperatorStickPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                m_robotDrive.zeroHeading();

                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> {
                                                        double leftY = WithDeadband(0.1,
                                                                        -m_driverController.getLeftY());
                                                        double leftX = WithDeadband(0.1,
                                                                        -m_driverController.getLeftX());
                                                        double rightX = WithDeadband(0.1,
                                                                        -m_driverController.getRightX());

                                                        m_robotDrive.drive(
                                                                        // Multiply by max speed to map the joystick
                                                                        // unitless inputs to actual units.
                                                                        // This will map the [-1, 1] to [max speed
                                                                        // backwards, max speed forwards],
                                                                        // converting them to actual units.
                                                                        leftY * DriveConstants.kMaxSpeedMetersPerSecond,
                                                                        leftX * DriveConstants.kMaxSpeedMetersPerSecond,
                                                                        rightX * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                                                                        false);
                                                },
                                                m_robotDrive));
                m_shooter.setDefaultCommand(
                                new RunCommand(
                                                () -> {
                                                        double stickY = m_operatorsStick.getRawAxis(1);
                                                        boolean shootButton = m_operatorsStick.getRawButton(1);
                                                        boolean suckButton = m_operatorsStick.getRawButton(5);
                                                        m_shooter.aimer(stickY);
                                                        if (shootButton) {
                                                                m_shooter.shootnNSuck(1);
                                                        } else if (suckButton) {
                                                                m_shooter.shootnNSuck(-1);
                                                        } else {
                                                                m_shooter.shootnNSuck(0);
                                                        }

                                                }, m_shooter));
                m_climber.setDefaultCommand(
                                new RunCommand(
                                                () -> {
                                                        boolean extendoButton = m_operatorsStick.getRawButton(2);
                                                        boolean retractoButton = m_operatorsStick.getRawButton(6);
                                                        if (extendoButton) {
                                                                m_climber.climber(1);
                                                        } else if (retractoButton) {
                                                                m_climber.climber(-1);
                                                        } else {
                                                                m_climber.climber(0);
                                                        }

                                                }, m_climber));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(DriveConstants.kDriveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(3, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(6, 1, new Rotation2d(0)),
                                config);

                var thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);

                // Reset odometry to the initial pose of the trajectory, run path following
                // command, then stop at the end.
                return Commands.sequence(
                                new InstantCommand(
                                                () -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
                                swerveControllerCommand,
                                new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
        }

        private static double WithDeadband(double deadband, double thumbstick) {
                return Math.abs(thumbstick) < deadband ? 0
                                : ((Math.abs(thumbstick) - deadband) / (1 - deadband) * Math.signum(thumbstick));
        }
}
