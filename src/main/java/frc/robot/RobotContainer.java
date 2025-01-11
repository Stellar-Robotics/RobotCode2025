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
import frc.robot.BaseConstants.DriveConstants;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotControl.StellarController;
import frc.robot.RobotControl.Commands.DriveWithRotaryCommand;
import frc.robot.RobotMechansims.AlgaeIntake.Subsystems.AlgaeIntake;
import frc.robot.RobotMechansims.PrimaryElevator.Subsystems.PrimaryElevator;
import frc.robot.RobotVision.VisionSubsystem;

public class RobotContainer {

  // Declare the robot chassis
  private SwerveChassisSubsystem chassis;

  // Declare Auto Selector
  private SendableChooser<Command> autoChooser;

  // Declare vision subsystem
  private VisionSubsystem vision;

  // Define Controllers
  public StellarController stellarController = new StellarController(0);


public RobotContainer() { 

  /* Initialization code is handled by calling initializeRobot in the Robot class */
  initiateRobot();
}

  /* This class is utilizing the singleton pattern
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


  public void initiateRobot() {
    /* ----------------------------------------------------------------------------------------
    *  The code below should be replaced with a command friendly method
    * of creating button binds, but for now, I'm too lazy. (PARTIALLY FIXED)
    * ---------------------------------------------------------------------------------------- */

    // Define the Chassis
    chassis = new SwerveChassisSubsystem();

    // Define vision subsystem
    vision = new VisionSubsystem(chassis.getPose());

    // Display the Auto Selector
    autoChooser = AutoBuilder.buildAutoChooser("Default");
    SmartDashboard.putData("Select Auto", autoChooser);

    // Display a speed control
    SmartDashboard.putNumber("TranslationSpeed", DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("RotationSpeed", DriveConstants.kMaxAngularSpeedFactor);

    // Configure default swerve controls
    chassis.setDefaultCommand(new DriveWithRotaryCommand(false, true, true, chassis));

    // Configure button binds
    configureButtonBinds();

    // Set vision periodic to run periodicly
    vision.setDefaultCommand(new RunCommand(() -> {vision.periodic();}, vision));

  }

  public Command getAutonomousCommand() {
    // Call the pathplanner auto lib
    return autoChooser.getSelected();
  }

  public void configureButtonBinds() {
    // Configure driver vision alt button
  }

  public Pose2d getVisionEstimate() {
    // Call the latest vision estimate
    return vision.getEstimatedGlobalPose();
  }
}
