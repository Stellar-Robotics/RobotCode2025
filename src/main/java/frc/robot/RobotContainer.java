// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseConstants.DriveConstants;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotControl.CommandStellarController;
import frc.robot.RobotChassis.Commands.DefaultDriveCommand;
import frc.robot.RobotChassis.Commands.SnapToReefCommand;
import frc.robot.RobotVision.VisionSubsystem;
import frc.robot.RobotControl.ControllerIO;
import frc.robot.RobotMechansims.CoralMech.Commands.IncramentCoralExtensionCommand;
import frc.robot.RobotMechansims.CoralMech.Commands.SetCoralMechPosition;
import frc.robot.RobotMechansims.CoralMech.Subsystems.CoralMech;
import frc.robot.RobotMechansims.Elevator.Commands.SetElevatorCommand;
import frc.robot.RobotMechansims.Elevator.Subsystems.Elevator;
import frc.robot.RobotMechansims.Elevator.Subsystems.Elevator.POSITIONS;

public class RobotContainer {

  // Declare subsystems
  private SwerveChassisSubsystem chassis; // Swerve subsystem
  private VisionSubsystem vision; // Vision subsystem
  private Elevator elevator; // Elevator subsystem
  private CoralMech coralMech; // Coral subsystem
  //private AlgaeMech algaeMech; // Algae subsystem
  // Declare controllers
  public CommandStellarController driverController = ControllerIO.getPrimaryInstance().stellarController;
  public CommandXboxController operatorController = ControllerIO.getSecondaryInstance().xboxController;

  // Declare Auto Selector
  private SendableChooser<Command> autoChooser;


  public RobotContainer() { 
    /* Initialization code is handled by calling initializeRobot in the Robot class */
    initiateRobot();
  }

  /* ------------------------------------------------------------------------------------------
   * This class is utilizing the singleton pattern
   * because there should be no other SubsystemContainer 
   * other than the one instance that this class hands out. */

  // Create singleton instance
  private static RobotContainer instance;

  // Create method for getting singleton instance.
  // If theres no instance, create one, then return the instance.
  public static RobotContainer getSingletonInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }
  // ------------------------------------------------------------------------------------------


  public void initiateRobot() {

    // Register commands with path planner
    bindCommandsToPathPlanner();

    // Define subsystems
    chassis = new SwerveChassisSubsystem(); // Swerve subsystem
    vision = new VisionSubsystem(chassis.getPose());
    elevator = new Elevator();
    coralMech = new CoralMech();
    //algaeMech = new AlgaeMech();

    // Create auto selector and post params to the dash
    autoChooser = AutoBuilder.buildAutoChooser("Default");
    SmartDashboard.putData("Select Auto", autoChooser);
    SmartDashboard.putNumber("TranslationSpeed", DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("RotationSpeed", DriveConstants.kMaxAngularSpeedFactor);

    // Configure default commands for subsystems
    chassis.setDefaultCommand(new DefaultDriveCommand(true, true, chassis));
    vision.setDefaultCommand(new RunCommand(() -> {vision.periodic();}, vision));

    // Bind buttons to the controllers
    configureButtonBinds();
  
  }


  public Command getAutonomousCommand() {
    // Call the pathplanner auto lib
    return autoChooser.getSelected();
  }

  public void configureButtonBinds() {
    // Bind commands to triggers

    // Switch to snapping mode
    operatorController.a().whileTrue(new SnapToReefCommand(chassis));

    // Incrament coral mech forward or backwards
    operatorController.rightBumper().onTrue(new IncramentCoralExtensionCommand(coralMech, true)).debounce(0.5);
    operatorController.leftBumper().onTrue(new IncramentCoralExtensionCommand(coralMech, false)).debounce(0.5);

    // Elevator presets
    operatorController.povUp().onTrue(new SetElevatorCommand(elevator, POSITIONS.HIGH)).debounce(0.1);
    operatorController.povLeft().or(operatorController.povRight()).onTrue(
      new SetElevatorCommand(elevator, POSITIONS.MID)
      .andThen(new SetCoralMechPosition(coralMech, 0)))
      .debounce(0.1);
    operatorController.povDown().onTrue(
      new SetElevatorCommand(elevator, POSITIONS.LOW)
      .andThen(new SetCoralMechPosition(coralMech, 0)))
      .debounce(0.1);

    // Run coral mechanism roller forward and backward.
    operatorController.rightTrigger().whileTrue(
      new RunCommand(() -> {coralMech.setRollerPower(operatorController.getHID().getRightTriggerAxis());}, coralMech)
    ).onFalse(
      new RunCommand(() -> {coralMech.setRollerPower(0);}, coralMech)
    );
    operatorController.leftTrigger().whileTrue(
      new RunCommand(() -> {coralMech.setRollerPower(-operatorController.getHID().getLeftTriggerAxis());}, coralMech)
    ).onFalse(
      new RunCommand(() -> {coralMech.setRollerPower(0);}, coralMech)
    );
  }

  public void bindCommandsToPathPlanner() {
    NamedCommands.registerCommand("ElevatorMedium", new SetElevatorCommand(elevator, POSITIONS.MID));
    NamedCommands.registerCommand("ElevatorHigh", new SetElevatorCommand(elevator, POSITIONS.HIGH));
    NamedCommands.registerCommand("ElevatorLow", new SetElevatorCommand(elevator, POSITIONS.LOW));
    System.out.println("Registered Commands With PathPlanner");
  }


  public Pose2d getVisionEstimate() {
    // Call the latest vision estimate
    return vision.getEstimatedGlobalPose();
  }
}
