// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotChassis.Commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotControl.ControllerIO;
import frc.robot.RobotControl.StellarController;
import frc.robot.RobotMechansims.MechanismConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoSnapCommand extends Command {
  SwerveChassisSubsystem chassis;
  int position;
  /** Creates a new AutoSnapCommand. */
  public AutoSnapCommand(SwerveChassisSubsystem chassisSubsystem, int position) {
    this.chassis = chassisSubsystem;
    this.position = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currPose = chassis.getPose();
    Pose2d closestPose = new Pose2d();
    double closestDistance = -1;

    Optional<Alliance> ally = DriverStation.getAlliance();

    if (ally.isPresent()) {

      // Get coord list based on alliance
      Pose2d[] coords; 
      if (ally.get() == Alliance.Blue) {
        coords = MechanismConstants.FieldNav.reefCoordsBlue;
      } else {
        coords = MechanismConstants.FieldNav.reefCoordsRed;
      }

      // Loop through coord list to find the closest coords
      for ( Pose2d pose : coords) {
        double dist = currPose.getTranslation().getDistance(pose.getTranslation());

        // Compare current closest position with new position
        // Or use new position as the current closest 
        // If no closest position has been established yet
        if (closestDistance >= 0) {
          boolean isCloser = dist < closestDistance;

          if (isCloser) {
            closestPose = pose;
            closestDistance = dist;
          }
          
        } else {
          closestPose = pose;
          closestDistance = dist;
        }
      }
    } else {
      System.out.println("Snapping Failed: No Alliance Found");
    }

    if (position == 1) {
      closestPose = closestPose.transformBy(new Transform2d(0, -0.165, new Rotation2d()));
    } else if (position == 2) {
      closestPose = closestPose.transformBy(new Transform2d(0, -0.33, new Rotation2d()));
    }

    System.out.println("Closest Pose: " + closestPose.toString());

    System.out.println("Poses:");
    System.out.println("Current Pose: " + currPose);
    System.out.println("Closest/Target Pose: " + closestPose);

    Pose2d rotatedPose = new Pose2d(currPose.getTranslation(), closestPose.getRotation());
    
    
    // Create list of waypoints from poses
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      currPose,
      rotatedPose,
      closestPose
    );

    System.out.println("Waypoints:");
    for (Waypoint wp : waypoints) {
        System.out.println(wp);
    }

    // Create new path from waypoint list
    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      MechanismConstants.FieldNav.snapConstraints, 
      null,
      new GoalEndState(0.0, closestPose.getRotation())
    );

    path.preventFlipping = true;

    StellarController driveController = ControllerIO.getPrimaryInstance().stellarController.getHID();

    double offset = closestPose.getRotation().getDegrees() - driveController.getRightRotary().getDegrees() + 180;
    RobotContainer.getSingletonInstance().setRotaryOffset(offset);
    SmartDashboard.putNumber("Rotary Offset", RobotContainer.getSingletonInstance().getRotaryOffset());

    // Follow the path
    AutoBuilder.followPath(path).schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
