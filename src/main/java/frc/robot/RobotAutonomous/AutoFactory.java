// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotAutonomous;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotChassis.Commands.AutoSnapCommand;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotMechansims.AlgaeMech.Subsystems.AlgaeMech;
import frc.robot.RobotMechansims.CoralMech.Commands.IncramentCoralExtensionCommand;
import frc.robot.RobotMechansims.CoralMech.Subsystems.CoralMech;
import frc.robot.RobotMechansims.Elevator.Commands.SetElevatorCommand;
import frc.robot.RobotMechansims.Elevator.Subsystems.Elevator;
import frc.robot.RobotMechansims.Elevator.Subsystems.Elevator.POSITIONS;

/** Add your docs here. */
public class AutoFactory {

    private SwerveChassisSubsystem chassis; 
    private Elevator elevator; 
    private CoralMech coral; 
    private AlgaeMech algae;

    private SendableChooser<String> startPositionChooser;
    private SendableChooser<String> faceChooser;
    private SendableChooser<Integer> positionChooser;
    private SendableChooser<POSITIONS> levelChooser;

    public AutoFactory(
        SwerveChassisSubsystem chassis, 
        Elevator elevator, 
        CoralMech coral, 
        AlgaeMech algae
    )
    {

        this.chassis = chassis;
        this.elevator = elevator;
        this.coral = coral;
        this.algae = algae;

        // Create options for starting position
        startPositionChooser.addOption("Left", "Left");
        startPositionChooser.addOption("Middle", "Middle");
        startPositionChooser.setDefaultOption("Right", "Right");
        // Create options for face
        faceChooser.setDefaultOption("Face D", "d");
        faceChooser.addOption("Face E", "e");
        faceChooser.addOption("Face F", "f");
        faceChooser.addOption("Face A", "a");
        faceChooser.addOption("Face B", "b");
        faceChooser.addOption("Face C", "c");
        // Create options for level
        levelChooser.setDefaultOption("L4", POSITIONS.HIGH);
        levelChooser.setDefaultOption("L3", POSITIONS.MID);
        levelChooser.setDefaultOption("L2", POSITIONS.LOW);
        // Create options for position
        positionChooser.addOption("Left", 0);
        positionChooser.addOption("Middle", 1);
        positionChooser.setDefaultOption("Right", 2);
        
        // Publish selectors
        SmartDashboard.putData("Select Starting Position", startPositionChooser);
        SmartDashboard.putData("Select Face", faceChooser);
        SmartDashboard.putData("Select Face Position", positionChooser);
        SmartDashboard.putData("Select Reef Level", levelChooser);

    }

    public PathPlannerPath getInitialPath(String face, String startPos) {
        // In order for this to work path must use the
        // following naming format: <face_letter_lowercase>From<start_position_firstuppercase>
        // Example: 'aFromLeft'
        try {
            return PathPlannerPath.fromPathFile(face + "From" + startPos);
        } catch (Exception e) {
            // Just fail for now
            return new PathPlannerPath(null, null, null, null);
        }
    }

    public Command lineupAndScore() {
        POSITIONS level = levelChooser.getSelected();
        Integer facePosition = positionChooser.getSelected();
        Command levelAndPosition;
        Command grabOrScore;


        levelAndPosition = new SequentialCommandGroup(
            new SetElevatorCommand(elevator, level),
            new AutoSnapCommand(chassis, facePosition)
        );

        if (facePosition == 1) {
            grabOrScore = new SequentialCommandGroup(
                algae.actuateExtension(false),
                algae.runPickup(1),
                new WaitCommand(2),
                algae.runPickup(0)
            );
        } else {
            grabOrScore = new SequentialCommandGroup(
                new IncramentCoralExtensionCommand(coral, true),
                new WaitCommand(1),
                coral.setRollerPower(1),
                new WaitCommand(0.7),
                coral.setRollerPower(0)
            );
        }

        return levelAndPosition.alongWith(grabOrScore);
    }

    public Command backupAndLower() {

        Pose2d currentPosition = chassis.getPose();
        Pose2d adjustedPosition = currentPosition.transformBy(new Transform2d(-0.5, 0, currentPosition.getRotation()));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        currentPosition,
        adjustedPosition
        );

        // Create new path from waypoint list
        PathPlannerPath path = new PathPlannerPath(
        waypoints,
        MechanismConstants.FieldNav.snapConstraints, 
        null,
        new GoalEndState(0.0, adjustedPosition.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path).andThen(new SequentialCommandGroup(
            new SetElevatorCommand(elevator, POSITIONS.LOW),
            new IncramentCoralExtensionCommand(coral, false)
        ));

    }

    public Command buildCustomAuto() {

        String selectedStartPos = startPositionChooser.getSelected();
        String selectedFace = faceChooser.getSelected();

        PathPlannerPath initialPath = getInitialPath(selectedFace, selectedStartPos);
        Command auto = AutoBuilder.followPath(initialPath).andThen(this.lineupAndScore()).andThen(this.backupAndLower());

        return auto;

    }

}
