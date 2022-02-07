// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.TRAJECTORY.*;


public class Trajectorydemo extends SubsystemBase {
  /** Creates a new Trajectorydemo. */
  public static WPI_TalonSRX keke = new WPI_TalonSRX(1);
  public Trajectorydemo() {
  }
  public void generateTrajectory(){
    //Traj 1
    var start = new Pose2d(1.2, 5.6, Rotation2d.fromDegrees(-180));
    var end = new Pose2d(23.3, 6.8, Rotation2d.fromDegrees(-160));

    var waypoints = new ArrayList<Translation2d>();
    waypoints.add(new Translation2d(2.2, 5.5));
    waypoints.add(new Translation2d(23, 18));

    TrajectoryConfig config = new TrajectoryConfig(maxVelo, maxAcce);
    config.setReversed(true);
    config.setStartVelocity(startVelo);
    config.setEndVelocity(endVelo1);
    

    
    var trajectory1 = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    
    //Traj 2
    var start2 = new Pose2d(1.2, 2.3, Rotation2d.fromDegrees(-140));
    var end2 = new Pose2d(3.4, 4.5, Rotation2d.fromDegrees(-120));

    var waypoints2 = new ArrayList<Translation2d>();
    waypoints2.add(new Translation2d(2.2, 5.5));
    waypoints2.add(new Translation2d(23, 18));

    TrajectoryConfig config2 = new TrajectoryConfig(3, 4);
    config.setReversed(true);
    config.setStartVelocity(endVelo1);
    config.setEndVelocity(endVelo2);

    
    var trajectory2 = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

    //Traj 3
    var trajectory3 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.1, 2.2, new Rotation2d(45)), 
      List.of(new Translation2d(1.1, 2.2), new Translation2d(3.3, 4.4)),
      new Pose2d(5.5, 6.6, new Rotation2d(-45)),
      new TrajectoryConfig(3, 4));

    var concat = trajectory1.concatenate(trajectory2).concatenate(trajectory3);

    double duration = concat.getTotalTimeSeconds();
    Trajectory.State point = concat.sample(1.2);

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Get the total time of the trajectory in seconds

  }
}
