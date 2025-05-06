package frc.robot.RobotChassis.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotContainer.REEFALIGNMENT;
import com.pathplanner.lib.path.Waypoint;
import java.util.List;

public class ChangeReefAlignment extends Command {
    
    SwerveChassisSubsystem chassis;
    REEFALIGNMENT relativeAlignment;
    REEFALIGNMENT absoluteAlignment;

    public ChangeReefAlignment(SwerveChassisSubsystem subsystem, REEFALIGNMENT alignment) {
        chassis = subsystem;
        relativeAlignment = alignment;
        absoluteAlignment = REEFALIGNMENT.LEFT;
    }

    void switchAlignment(Transform2d transform) {
        Pose2d newPose = chassis.getPose().transformBy(transform);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            chassis.getPose(),
            newPose
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            MechanismConstants.FieldNav.snapConstraints,
            null,
            new GoalEndState(0.0, chassis.getPose().getRotation())
        );

        RobotContainer.getSingletonInstance().setReefAlignment(absoluteAlignment);

        // Follow the path
        AutoBuilder.followPath(path).schedule();
    }

    REEFALIGNMENT findAbsoluteAlignment(REEFALIGNMENT alignment) {
        if (alignment == REEFALIGNMENT.RIGHT) {
            if (RobotContainer.getSingletonInstance().getReefAlignment() == REEFALIGNMENT.LEFT) {
                return REEFALIGNMENT.CENTER;
            } else if (RobotContainer.getSingletonInstance().getReefAlignment() == REEFALIGNMENT.CENTER) {
                return REEFALIGNMENT.RIGHT;
            }
        } else if (alignment == REEFALIGNMENT.LEFT) {
            if (RobotContainer.getSingletonInstance().getReefAlignment() == REEFALIGNMENT.RIGHT) {
                return REEFALIGNMENT.CENTER;
            } else if (RobotContainer.getSingletonInstance().getReefAlignment() == REEFALIGNMENT.CENTER) {
                return REEFALIGNMENT.LEFT;
            }
        }
        return REEFALIGNMENT.LEFT;
    }

    @Override
    public void execute() {
        // Find absolute reef alignment from relative
        absoluteAlignment = findAbsoluteAlignment(relativeAlignment);

        // Convert the a
        if (relativeAlignment == REEFALIGNMENT.CENTER) {
            if (RobotContainer.getSingletonInstance().getReefAlignment() == REEFALIGNMENT.LEFT) {
                Transform2d transform = new Transform2d(0.165, 0, new Rotation2d());
                switchAlignment(transform);
            } else if (RobotContainer.getSingletonInstance().getReefAlignment() == REEFALIGNMENT.RIGHT) {
                Transform2d transform = new Transform2d(-0.165, 0, new Rotation2d());
                switchAlignment(transform);
            }
        } else if (relativeAlignment == REEFALIGNMENT.LEFT) {
            if (RobotContainer.getSingletonInstance().getReefAlignment() == REEFALIGNMENT.CENTER) {
                Transform2d transform = new Transform2d(-0.165, 0, new Rotation2d());
                switchAlignment(transform);
            } else if (RobotContainer.getSingletonInstance().getReefAlignment() == REEFALIGNMENT.RIGHT) {
                Transform2d transform = new Transform2d(-0.33, 0, new Rotation2d());
                switchAlignment(transform);
            }
        } else {
            if (RobotContainer.getSingletonInstance().getReefAlignment() == REEFALIGNMENT.CENTER) {
                Transform2d transform = new Transform2d(0.165, 0, new Rotation2d());
                switchAlignment(transform);
            } else if (RobotContainer.getSingletonInstance().getReefAlignment() == REEFALIGNMENT.LEFT) {
                Transform2d transform = new Transform2d(0.33, 0, new Rotation2d());
                switchAlignment(transform);
            }
        }
    }
}
