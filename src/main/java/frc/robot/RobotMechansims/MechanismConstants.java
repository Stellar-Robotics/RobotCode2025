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
            // Each position corresponds to the branch to the robot's left 
            // When the robot is facing the respective face

            // Rotations are an estimate, be careful

            // Blue 1
            new Pose2d(3.097, 4.200, new Rotation2d()),
            // Blue 2
            new Pose2d(3.664, 2.902, Rotation2d.fromDegrees(60)),
            // Blue 3
            new Pose2d(5.052, 2.715, Rotation2d.fromDegrees(120)), 
            // Blue 4
            new Pose2d(5.887, 3.865, Rotation2d.fromDegrees(180)),
            // Blue 5
            new Pose2d(5.328, 5.155, Rotation2d.fromDegrees(-120)),
            // Blue 6
            new Pose2d(3.940, 5.335, Rotation2d.fromDegrees(-60)),
        };

        public static final Pose2d[] reefCoordsRed = {

            // Red 1
            new Pose2d(11.640, 4.193, Rotation2d.fromDegrees(0)),
            // Red 2
            new Pose2d(12.222, 2.887, Rotation2d.fromDegrees(60)),
            // Red 3
            new Pose2d(13.610, 2.738, Rotation2d.fromDegrees(120)),
            // Red 4
            new Pose2d(14.483, 3.865, Rotation2d.fromDegrees(180)),
            // Red 5
            new Pose2d(13.909, 5.155, Rotation2d.fromDegrees(-120)),
            // Red 6
            new Pose2d(12.521, 5.305, Rotation2d.fromDegrees(-60)),
        };

        // Constraints to use when snaping to position at reef
        public static final PathConstraints snapConstraints = new PathConstraints(
            1.5, 
            4.0,
            Units.degreesToRadians(540), 
            Units.degreesToRadians(720)
        );
    }

    public class elevatorValues {
        public static final int motorID = 19;
        public static final double maxHeightExtensionRotations = 178;

        public static enum ELEVATORPOSITION {
            LOW,
            MID,
            HIGH
        };

        public static ELEVATORPOSITION currentPos = ELEVATORPOSITION.LOW;
    }

    public class ClimberValues {
        public static final int motorID1 = 13;
        public static final int motorID2 = 14;

        public static boolean triggered = false;
    }

    public class CoralMechValues {
        public static final int rollerMotorID = 10;
        public static final int extensionMotorID = 11;
        public static final int extensionMotorID2 = 12;

        public static final double maxExtension = 43;
        public static final double minExtension = -45;

        public static enum CORALEXTENSIONPOSITION {
            BACK,
            MIDDLE,
            FORWARD,
            XBACK
        };

        public static boolean lastPos = false;

        public static CORALEXTENSIONPOSITION currentPos = CORALEXTENSIONPOSITION.MIDDLE;
    }

    public class AlgaeMechValues {
        public static final int pickupMotorID = 15;
    }

}
