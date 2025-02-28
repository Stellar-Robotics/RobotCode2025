// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Constants file for holding vision id and positioning information */
public class VisionConstants {

    // THE METHOD FOR STORING VISION CONSTANTS HAS NOT BEEN FULLY
    // DETERMINED YET AND WILL MOST LIKELY CHANGE A LOT OVER TIME

    // Store data specific to photonvision
    public static final class PhotonConstants {

        /* These names are the same as the camera name in
         * chosen the photon client for each camera and
         * are identified as so in network tables */ 
        public static final String cameraName2 = "StellarCam1";
        public static final String cameraName1 = "StellarCam2";

    }

    public static final class PositionConstants {

        // Camera 1 position relative to the robot center ([Facing Backward In Center][Quarter and a half Meter Backwards][A Quarter Meter Up][Tilted Upward Half A Radian]) (+X is front of robot)
        //public static final Transform3d robotToCam = new Transform3d(new Translation3d(-0.35, 0, 0.25), new Rotation3d(0, -0.535 * Math.PI, Math.toRadians(180)));
        //public static final Transform3d robotToCam = new Transform3d(new Translation3d(-0.5/*-0.3524*/, 0, 0.2318), new Rotation3d(0, Math.toRadians(-32), Math.toRadians(180)));

        //public static final Transform3d reefCamTransform = new Transform3d(new Translation3d(0.081, -0.2572, 0.4413), new Rotation3d(0, 0, Math.toRadians(160.940)));
        public static final Transform3d reefCamTransform = new Transform3d(new Translation3d(-0.081, 0.2572, 0.4413), new Rotation3d(0, 0, Math.toRadians(340.94)));


        public static final Transform3d generalCamTransform = new Transform3d(new Translation3d(-0.2762, 0.1683, 0.7461), new Rotation3d(90, Math.toRadians(0), Math.toRadians(174.5)));

        // Get apriltag position data via FIRST provided json file (ChangeMe for 2025)
        public static final AprilTagFieldLayout tagPositions = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);


    }

}
