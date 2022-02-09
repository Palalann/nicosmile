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
import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import static frc.robot.Constants.TRAJECTORY.*;
import static frc.robot.Constants.PATH1.*;
import static frc.robot.Constants.PATH2.*;
import static frc.robot.Constants.PATH3.*;
import java.util.ArrayList;




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

    //Traj 1    
    var start = new Pose2d(start1_x,start1_y, Rotation2d.fromDegrees(start1_thetha));
    var end = new Pose2d(end1_x, end1_y, Rotation2d.fromDegrees(end1_thetha));

    var waypoints = new ArrayList<Translation2d>();
    waypoints.add(new Translation2d(point11_x, point11_y));
    waypoints.add(new Translation2d(point12_x, point12_y));
    waypoints.add(new Translation2d(point13_x, point13_y));
    waypoints.add(new Translation2d(point14_x, point14_y));    

    TrajectoryConfig config = new TrajectoryConfig(maxVelo, maxAcce);
    config.setStartVelocity(startVelo);
    config.setEndVelocity(endVelo1);
        
    var trajectory1 = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    
    ////////////////////////////////////////////////////////
    //Traj 2
    var start2 = new Pose2d(start2_x,start2_y, Rotation2d.fromDegrees(start2_thetha));
    var end2 = new Pose2d(end2_x, end2_y, Rotation2d.fromDegrees(end2_thetha));

    var waypoints2 = new ArrayList<Translation2d>();
    waypoints.add(new Translation2d(point21_x, point21_y));
    waypoints.add(new Translation2d(point22_x, point22_y));
    waypoints.add(new Translation2d(point23_x, point23_y));
    waypoints.add(new Translation2d(point24_x, point24_y));
    waypoints.add(new Translation2d(point25_x, point25_y));

    TrajectoryConfig config2 = new TrajectoryConfig(maxVelo, maxAcce);
    config2.setReversed(isReverse2);

    config2.setReversed(isReverse2);
    config2.setStartVelocity(endVelo1);
    config2.setEndVelocity(endVelo2);

    
    var trajectory2 = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

    /////////////////////////////////////////////////////////
    //Traj 3
    var trajectory3 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(start3_x, start3_y, new Rotation2d(start3_thetha)), 
      List.of(new Translation2d(point31_x, point31_y), 
              new Translation2d(point32_x, point32_y)),
      new Pose2d(end3_x, end3_y, new Rotation2d(end3_thetha)),
      new TrajectoryConfig(maxVelo, maxAcce));

    var concat = trajectory1.concatenate(trajectory2).concatenate(trajectory3);

    double duration = concat.getTotalTimeSeconds();
    Trajectory.State point = concat.sample(1.2);


    ////////////////////////////////////////////////////////
    RamseteController m_disabledRamsete = new RamseteController();
    m_disabledRamsete.setEnabled(false);
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            concat,
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
    drivetrain.resetOdometry(concat.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
 
 }
 


}

