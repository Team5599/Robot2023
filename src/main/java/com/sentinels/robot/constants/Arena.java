/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/
package com.sentinels.robot.constants;

import java.nio.file.Path;
import java.util.List;

import com.sentinels.robot.constants.Settings.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public interface Arena {
    public interface Trajectories{
        // use this interface to set the trajectories that the robot will use in auto which it will follow with the ramsete controller

        // SIMULATION TESTS
        public TrajectoryConfig kConfig = new TrajectoryConfig(
            Units.feetToMeters(3.0), Units.feetToMeters(3.0))
            .setKinematics(Drivetrain.KINEMATICS
        );

        public Trajectory TestTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0))
        );

        public Trajectory SimpleTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, Rotation2d.fromDegrees(0)), //starting postion
            List.of(new Translation2d(3,0)), //shows the turning process thingy
            new Pose2d(3,3, Rotation2d.fromDegrees(90)),
            kConfig
        );

        public Trajectory StraightLine = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(3, 0)), 
            new Pose2d(6, 0, Rotation2d.fromDegrees(0)), 
            kConfig
        );

        // JSONs / REAL 
        //https://docs.wpilib.org/en/stable/docs/software/pathplanning/pathweaver/integrating-robot-program.html

        
        public enum Routine0{
            ToCube1("ToCube1.wpilib.json"),
            ToCube2("ToCube2.wpilib.json");

            String path;
            public Trajectory trajectory;
            Routine0(String name){
                path = "output/" + name;

                try {
                    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path); 
                    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                } catch (Exception e) {
                    DriverStation.reportError("UNABLE TO OPEN TRAJECTORY" + path, e.getStackTrace());
                }
            }
        }
    }
    public interface Keypoints{
        // use this interface to set the coordinates of points around the arena, like the charging dock, gamepieces and human player portal, etc
    }
}
