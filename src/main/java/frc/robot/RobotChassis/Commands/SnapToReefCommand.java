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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotMechansims.MechanismConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapToReefCommand extends Command {
  /** Creates a new SnapToReefCommand. */
  SwerveChassisSubsystem chassis;

  /**  
   * <pre>
   * Snaps the robot to the nearest reef face
   * 
   *This command currently only snaps to the left side of each reef face
   * </pre>
  */
  public SnapToReefCommand(SwerveChassisSubsystem chassisSubsystem) {
    addRequirements(chassisSubsystem);
    this.chassis = chassisSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // This command currently only snaps to the left side of each reef face
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
    
    // Create list of waypoints from poses
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      currPose,
      closestPose
    );

    // Create new path from waypoint list
    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      MechanismConstants.FieldNav.snapConstraints, 
      null, 
      new GoalEndState(0.0, closestPose.getRotation())
    );

    // Follow the path
    AutoBuilder.followPath(path).execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
