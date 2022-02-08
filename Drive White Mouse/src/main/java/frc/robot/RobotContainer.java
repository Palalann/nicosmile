// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Differentialdb;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.MEASUREMENT.*;
import static frc.robot.Constants.RAMSETE.*;
import static frc.robot.Constants.TRAJECTORY.*;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.Joystick;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Differentialdb drivetrain = new Differentialdb();
  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACKWIDTH);
  public static final Joystick controller = new Joystick(1);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(ks,kv,ka),
      kinematics, 10);

    // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(maxVelo, maxAcce);
            config.setReversed(true);
            config.setStartVelocity(startVelo);
            config.setEndVelocity(endVelo1);
    
            // Add kinematics to ensure max speed is actually obeyed
            config.setKinematics(kinematics);
            // Apply the voltage constraint
            config .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    //Traj 1
    var start = new Pose2d(1.2, 5.6, Rotation2d.fromDegrees(-180));
    var end = new Pose2d(23.3, 6.8, Rotation2d.fromDegrees(-160));

    var waypoints = new ArrayList<Translation2d>();
    waypoints.add(new Translation2d(2.2, 5.5));
    waypoints.add(new Translation2d(23, 18));
    
    var trajectory1 = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory1,
            drivetrain::getPose,
            new RamseteController(kRamseteB, kRamseteZeta),
            new SimpleMotorFeedforward(
                ks,kv, ka),kinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(kP, 0, 0),
            new PIDController(kP, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts,
            drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    drivetrain.resetOdometry(trajectory1.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
 
 }

}

