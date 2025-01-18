// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class MechanismConstants {

    // Holds strategic positions for the robot to snap to
    public class FieldNav {
        public static final Pose2d[] reefCoordsBlue = {

            // Ordered starting clostest to origin Y and progrssing CCW

            // Blue 1
            new Pose2d(3.353, 4.026, new Rotation2d()),
            // Blue 2
            new Pose2d(3.921, 3.042, new Rotation2d()),
            // Blue 3
            new Pose2d(5.058, 3.042, new Rotation2d()),
            // Blue 4
            new Pose2d(5.626, 4.026, new Rotation2d()),
            // Blue 5
            new Pose2d(5.058, 5.010, new Rotation2d()),
            // Blue 6
            new Pose2d(3.921, 5.010, new Rotation2d()),
        };

        public static final Pose2d[] reefCoordsRed = {

            // Red 1
            new Pose2d(11.922, 4.026, new Rotation2d()),
            // Red 2
            new Pose2d(12.491, 3.042, new Rotation2d()),
            // Red 3
            new Pose2d(13.627, 3.042, new Rotation2d()),
            // Red 4
            new Pose2d(14.195, 4.026, new Rotation2d()),
            // Red 5
            new Pose2d(13.627, 5.010, new Rotation2d()),
            // Red 6
            new Pose2d(12.491, 5.010, new Rotation2d()),
        };

        // Constraints to use when snaping to position at reef
        public static final PathConstraints snapConstraints = new PathConstraints(
            3.0, 
            4.0,
            Units.degreesToRadians(540), 
            Units.degreesToRadians(720)
        );
    }

}
