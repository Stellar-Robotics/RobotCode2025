// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BaseConstants.DriveConstants;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotControl.CommandStellarController;
import frc.robot.RobotChassis.Commands.DefaultDriveCommand;
import frc.robot.RobotVision.VisionSubsystem;
import frc.robot.RobotControl.ControllerIO;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotMechansims.MechanismConstants.CoralMechValues.CORALEXTENSIONPOSITION;
import frc.robot.RobotMechansims.MechanismConstants.elevatorValues.ELEVATORPOSITION;
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

  // Pneumatics
  private PneumaticHub pneumaticHub;
  private DoubleSolenoid climberLock;
  
  // Declare controllers
  public CommandStellarController driverController = ControllerIO.getPrimaryInstance().stellarController;
  public CommandXboxController operatorController = ControllerIO.getSecondaryInstance().xboxController;

  // Declare Auto Selector
  private SendableChooser<Command> autoChooser;

  private double rotaryOffset;
  //private boolean snapping;

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

    // Pneumatic Hub
    pneumaticHub = new PneumaticHub(18);
    pneumaticHub.enableCompressorDigital();
    climberLock = pneumaticHub.makeDoubleSolenoid(1, 2);

    // Define subsystems
    chassis = new SwerveChassisSubsystem(); // Swerve subsystem
    vision = new VisionSubsystem(chassis.getPose());
    elevator = new Elevator();
    coralMech = new CoralMech();
    climber = new ClimbSubsystem(climberLock);

    rotaryOffset = 0;

    // Create auto selector and post params to the dash
    SmartDashboard.putNumber("TranslationSpeed", DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("RotationSpeed", DriveConstants.kMaxAngularSpeedFactor);

    bindCommandsToPathPlanner();

    // Configure default commands for subsystems
    chassis.setDefaultCommand(new DefaultDriveCommand(true, true, chassis));

    // Bind commands to the controllers and pathplanner
    configureButtonBinds();
    chassis.initAutoBuilder();

    // Build autosP
    autoChooser = AutoBuilder.buildAutoChooser("Default");
    SmartDashboard.putData("Select Auto", autoChooser);
  
  }


  public Command getAutonomousCommand() {
      return autoChooser.getSelected().withTimeout(15);
  }

  public void setRotaryOffset(double offset) {
    this.rotaryOffset = offset;
  }

  public double getRotaryOffset() {
    return this.rotaryOffset;
  }

  public double getElevatorPosition() {
    return elevator.getPosition();
  }

  public void configureButtonBinds() {

    // ____________________________________________________________________________________________
    // Speed control

    driverController.rightPaddle().whileTrue(
      new RunCommand(() -> {
        if (chassis.rampActive) {
          chassis.rampActive = false;
        }
        DriveConstants.paddleSpeedOverride = 0.3;

      })
    );

    driverController.rightPaddle().onFalse(
      Commands.runOnce(() -> {
        chassis.rampActive = true;
      }, chassis)
    );
    // _____________________________________________________________________________________________
    // Incrament coral mech forward or backwards

    operatorController.rightBumper().onTrue(new IncramentCoralExtensionCommand(coralMech, true)).debounce(0.5);
    operatorController.rightTrigger().onTrue(new IncramentCoralExtensionCommand(coralMech, false)).debounce(0.5);
    operatorController.b().onTrue(Commands.runOnce(() -> {coralMech.goToPosition(-43); 
      MechanismConstants.CoralMechValues.currentPos = CORALEXTENSIONPOSITION.XBACK;
    }, coralMech));
    // _____________________________________________________________________________________________
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
        .andThen(Commands.runOnce(() -> MechanismConstants.CoralMechValues.currentPos = CORALEXTENSIONPOSITION.FORWARD, coralMech)),
        new SetCoralMechPosition(coralMech, 0, true)
        .andThen(new WaitCommand(0.5))
        .andThen(new SetElevatorCommand(elevator, POSITIONS.MID)),
        () -> { return MechanismConstants.elevatorValues.currentPos == ELEVATORPOSITION.LOW; }))
      .debounce(0.1);
      new Trigger(() -> operatorController.getRightY() < -0.5).onTrue(
        new SetElevatorCommand(elevator, POSITIONS.ALGAEHIGH)
      ).debounce(0.1);
      new Trigger(() -> operatorController.getRightY() > 0.5).onTrue(
        new SetElevatorCommand(elevator, POSITIONS.ALGAELOW)
    ).debounce(0.1);
    operatorController.povDown().onTrue(
      new SetCoralMechPosition(coralMech, 0, true)
      .andThen(new WaitCommand(0.5))
      .andThen(new SetElevatorCommand(elevator, POSITIONS.LOW))
      ).debounce(0.1);
    // ______________________________________________________________________________________________
    // Run coral mechanism roller forward and backward

    operatorController.leftBumper() // Run forward
    .whileTrue(coralMech.setRollerPower(1))
    .onFalse(coralMech.setRollerPower(0));
    operatorController.leftTrigger() // Run backward
    .whileTrue(coralMech.setRollerPower(-1))
    .onFalse(coralMech.setRollerPower(0));
    // _______________________________________________________________________________________________
    // Climbing

    operatorController.x().onTrue(
      new SequentialCommandGroup(
        // elevator.GoToClimbPosition(), // Raise the elevator
        coralMech.goFullBack(), // Send the coral mechanism to the corner
        climber.toggleLock(climber, 1), // Ensure the climber is unlocked
        new WaitCommand(1), // Wait for the elevator to clear
        climber.setClimber(elevator.getPosition(), false) // Bring the climber up
      ));
    driverController.rightTop().onTrue(Commands.runOnce(() -> {
      climber.setPosition(0);
    }, climber));
    driverController.leftTop().onTrue(climber.engageAndLock(elevator.getPosition())); // Commit to climb
    // _______________________________________________________________________________________________

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
    NamedCommands.registerCommand("coralPickup", coralMech.goFullBack());
    NamedCommands.registerCommand("runCoral", coralMech.runCoral(1));

    // Other Commands
    NamedCommands.registerCommand("scoreCoral", scoreAndResetCommand());
    
    // Debugging info
    System.out.println("Registered Commands With PathPlanner");
  }


  public Pose2d getVisionEstimate(boolean reef) {
    // Call the latest vision estimate
    return vision.getEstimatedGlobalPose(reef);
  }


  // __________________________________________________________________________________
  // Commands

  public Command scoreAndResetCommand() {
    return coralMech.runCoral(0.75)
    .andThen(new IncramentCoralExtensionCommand(coralMech, false))
    .andThen(new SetElevatorCommand(elevator, POSITIONS.LOW));
  }

}
