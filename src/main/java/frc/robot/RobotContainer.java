// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BaseConstants.DriveConstants;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotControl.CommandStellarController;
import frc.robot.RobotChassis.Commands.ChangeReefAlignment;
import frc.robot.RobotChassis.Commands.DefaultDriveCommand;
import frc.robot.RobotChassis.Commands.SnapToReefCommand;
import frc.robot.RobotVision.VisionSubsystem;
import frc.robot.RobotControl.ControllerIO;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotMechansims.MechanismConstants.CoralMechValues.CORALEXTENSIONPOSITION;
import frc.robot.RobotMechansims.MechanismConstants.elevatorValues.ELEVATORPOSITION;
import frc.robot.RobotMechansims.ClimbMech.Commands.TriggerClimberCommand;
import frc.robot.RobotMechansims.ClimbMech.Subsystems.ClimbSubsystem;
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
  private ClimbSubsystem climber; // Climber subsystem

  public static enum REEFALIGNMENT {
    LEFT,
    CENTER,
    RIGHT
  }

  //private AlgaeMech algaeMech; // Algae subsystem
  
  // Declare controllers
  public CommandStellarController driverController = ControllerIO.getPrimaryInstance().stellarController;
  public CommandXboxController operatorController = ControllerIO.getSecondaryInstance().xboxController;

  // Declare Auto Selector
  private SendableChooser<Command> autoChooser;

  private double rotaryOffset;
  private REEFALIGNMENT currentReefAlignment;
  private boolean snapping;

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
    //bindCommandsToPathPlanner();

    // Define subsystems
    chassis = new SwerveChassisSubsystem(); // Swerve subsystem
    vision = new VisionSubsystem(chassis.getPose());
    elevator = new Elevator();
    coralMech = new CoralMech();
    climber = new ClimbSubsystem();
    //algaeMech = new AlgaeMech();

    rotaryOffset = 0;
    currentReefAlignment =  REEFALIGNMENT.LEFT;
    snapping = false;

    // Create auto selector and post params to the dash
    SmartDashboard.putNumber("TranslationSpeed", DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("RotationSpeed", DriveConstants.kMaxAngularSpeedFactor);

    bindCommandsToPathPlanner();

    // Configure default commands for subsystems
    chassis.setDefaultCommand(new DefaultDriveCommand(true, true, chassis));
    vision.setDefaultCommand(new RunCommand(() -> {vision.periodic();}, vision));

    // Bind commands to the controllers and pathplanner
    configureButtonBinds();
    chassis.initAutoBuilder();

    // Build autosP
    autoChooser = AutoBuilder.buildAutoChooser("Default");
    SmartDashboard.putData("Select Auto", autoChooser);
  
  }


  public Command getAutonomousCommand() {
    // Call the pathplanner auto lib
    return autoChooser.getSelected();
  }

  public void setRotaryOffset(double offset) {
    this.rotaryOffset = offset;
  }

  public double getRotaryOffset() {
    return this.rotaryOffset;
  }

  public void setReefAlignment(REEFALIGNMENT alignment) {
    this.currentReefAlignment = alignment;
  }

  public REEFALIGNMENT getReefAlignment() {
    return this.currentReefAlignment;
  } 

  public void configureButtonBinds() {
    // Bind commands to triggers

    // Switch to snapping mode
    operatorController.a().onTrue(
      new SnapToReefCommand(chassis)
      .andThen(new RunCommand(() -> currentReefAlignment = REEFALIGNMENT.LEFT))
      .andThen(new RunCommand(() -> snapping = true))
    )
    .debounce(0.1);

    operatorController.a().onFalse(
      new RunCommand(() -> snapping = false)
    )
    .debounce(0.1);

    // Incrament coral mech forward or backwards
    operatorController.rightBumper().onTrue(new IncramentCoralExtensionCommand(coralMech, true)).debounce(0.5);
    operatorController.rightTrigger().onTrue(
      new ConditionalCommand(
        new ChangeReefAlignment(chassis, REEFALIGNMENT.RIGHT),
        new IncramentCoralExtensionCommand(coralMech, false), 
        () -> snapping))
      .debounce(0.5);
    operatorController.b().onTrue(new RunCommand(() -> { coralMech.goToPosition(-43); 
      MechanismConstants.CoralMechValues.currentPos = CORALEXTENSIONPOSITION.XBACK;
    }, coralMech));

    // Elevator presets
    operatorController.povUp().onTrue(
      new SetElevatorCommand(elevator, POSITIONS.HIGH)
      .andThen(new WaitCommand(2))
      .andThen(new SetCoralMechPosition(coralMech, 34, false))
      .andThen(new RunCommand(() -> MechanismConstants.CoralMechValues.currentPos = CORALEXTENSIONPOSITION.FORWARD)))
      .debounce(0.1);
    operatorController.povLeft().or(operatorController.povRight()).onTrue(
      new ConditionalCommand(
        new SetElevatorCommand(elevator, POSITIONS.MID)
        .andThen(new WaitCommand(0.5))
        .andThen(new SetCoralMechPosition(coralMech, 34, false))
        .andThen(new RunCommand(() -> MechanismConstants.CoralMechValues.currentPos = CORALEXTENSIONPOSITION.FORWARD)),
        new SetCoralMechPosition(coralMech, 0, true)
        .andThen(new WaitCommand(0.5))
        .andThen(new SetElevatorCommand(elevator, POSITIONS.MID)),
        () -> { return MechanismConstants.elevatorValues.currentPos == ELEVATORPOSITION.LOW; }))
      .debounce(0.1);
    operatorController.povDown().onTrue(
      new SetCoralMechPosition(coralMech, 0, true)
      .andThen(new WaitCommand(0.5))
      .andThen(new SetElevatorCommand(elevator, POSITIONS.LOW)))
      .debounce(0.1);

    // Run coral mechanism roller forward and backward.
    operatorController.leftBumper().whileTrue(
      new RunCommand(() -> {coralMech.setRollerPower(1);}, coralMech)
    ).onFalse(
      new RunCommand(() -> {coralMech.setRollerPower(0);}, coralMech)
    );
    operatorController.leftTrigger().whileTrue(
      new ConditionalCommand(
        new ChangeReefAlignment(chassis, REEFALIGNMENT.RIGHT),
        new RunCommand(() -> {coralMech.setRollerPower(-1);}, coralMech), 
        () -> snapping)
    ).onFalse(
      new RunCommand(() -> {coralMech.setRollerPower(0);}, coralMech)
    );

    // Deploy the machettis!
    operatorController.back().onTrue(
      new TriggerClimberCommand(climber)
    );

    // FOR TESTING
    operatorController.start().onTrue(
      new RunCommand(() -> {
        MechanismConstants.ClimberValues.triggered = false;
        climber.setSpeed(0);
      }, climber)
    );
  }

  public void bindCommandsToPathPlanner() {
    // Auto command bindings

    // Elevator
    NamedCommands.registerCommand("elevatorMedium", new SetElevatorCommand(elevator, POSITIONS.MID));
    NamedCommands.registerCommand("elevatorHigh", new SetElevatorCommand(elevator, POSITIONS.HIGH));
    NamedCommands.registerCommand("elevatorLow", new SetElevatorCommand(elevator, POSITIONS.LOW));

    // Coral
    NamedCommands.registerCommand("coralForward", new IncramentCoralExtensionCommand(coralMech, true));
    NamedCommands.registerCommand("coralBackward", new IncramentCoralExtensionCommand(coralMech, false));

    // Other
    NamedCommands.registerCommand("snapToReefCommand", new SnapToReefCommand(chassis));

    // Path command bindings
    
    // Elevator
    new EventTrigger("ElevatorMedium").onTrue(new SetElevatorCommand(elevator, POSITIONS.MID));
    new EventTrigger("ElevatorHigh").onTrue(new SetElevatorCommand(elevator, POSITIONS.HIGH));
    new EventTrigger("ElevatorLow").onTrue(new SetElevatorCommand(elevator, POSITIONS.LOW));

    // Coral
    new EventTrigger("coralForward").onTrue(new IncramentCoralExtensionCommand(coralMech, true));
    new EventTrigger("coralBackward").onTrue(new IncramentCoralExtensionCommand(coralMech, false));

    System.out.println("Registered Commands With PathPlanner");
  }


  public Pose2d getVisionEstimate(boolean reef) {
    // Call the latest vision estimate
    return vision.getEstimatedGlobalPose(reef);
  }
}
