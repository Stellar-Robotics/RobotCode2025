// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

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
import frc.robot.RobotVision.VisionSubsystem;
import frc.robot.RobotControl.ControllerIO;


public class RobotContainer {

  // Declare subsystems
  private SwerveChassisSubsystem chassis; // Swerve subsystem
  private VisionSubsystem vision; // Vision subsystem

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

    // Define subsystems
    chassis = new SwerveChassisSubsystem(); // Swerve subsystem
    vision = new VisionSubsystem(chassis.getPose());

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

    operatorController.a().whileTrue( 
        new RunCommand(() -> {
          
        })
    );

    // operatorController.povUp().onTrue( // Incrament elevator preset (up)

    // ).debounce(0.2);

    // operatorController.povDown().onTrue( // Incrament elevator preset (down)

    // ).debounce(0.2);

    // operatorController.leftBumper().whileTrue( // Run algae intake inward

    // );

    // operatorController.leftTrigger(0.5).whileTrue( // Run algae intake outward
    
    // );
  }


  public Pose2d getVisionEstimate() {
    // Call the latest vision estimate
    return vision.getEstimatedGlobalPose();
  }
}
