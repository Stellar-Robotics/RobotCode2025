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

            // Ordered starting clostest to origin Y and progressing CCW
            // Positions probably intersect with the reef, will need to fix later

            // Each branch is 6.5 inches from the center of the reef face

            // Rotations are an estimate, be careful

            // Blue 1
            new Pose2d(3.353, 4.026, new Rotation2d()),
            // Blue 2
            new Pose2d(3.921, 3.042, new Rotation2d(60)),
            // Blue 3
            new Pose2d(5.058, 3.042, new Rotation2d(120)),
            // Blue 4
            new Pose2d(5.626, 4.026, new Rotation2d(180)),
            // Blue 5
            new Pose2d(5.058, 5.010, new Rotation2d(-120)),
            // Blue 6
            new Pose2d(3.921, 5.010, new Rotation2d(-60)),
        };

        public static final Pose2d[] reefCoordsRed = {

            // Red 1
            new Pose2d(11.922, 4.026, new Rotation2d(0)),
            // Red 2
            new Pose2d(12.491, 3.042, new Rotation2d(60)),
            // Red 3
            new Pose2d(13.627, 3.042, new Rotation2d(120)),
            // Red 4
            new Pose2d(14.195, 4.026, new Rotation2d(180)),
            // Red 5
            new Pose2d(13.627, 5.010, new Rotation2d(-120)),
            // Red 6
            new Pose2d(12.491, 5.010, new Rotation2d(-60)),
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
