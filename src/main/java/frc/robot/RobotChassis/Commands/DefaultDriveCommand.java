// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotChassis.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BaseConstants;
import frc.robot.BaseConstants.DriveConstants;
import frc.robot.BaseConstants.IOConstants;
import frc.robot.BaseConstants.MiscConstants;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotControl.ControllerIO;
import frc.robot.RobotControl.StellarController;
import frc.robot.RobotUtilities.MiscUtils;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDriveCommand extends Command {

  // Create variables for the arguments passed in so they can be refrenced class wide
  private SwerveChassisSubsystem chassis;
  private boolean useDeadband;
  private boolean fieldRelative;

  // Base rotation multiplier
  private double m_currentRotation = 0.0;

  // Define the PID for the rotary dial
  private PIDController rotaryPID = new PIDController(0.01, 0 , 0);


  /** Creates a new DriveWithRotary. */
  public DefaultDriveCommand(Boolean useDeadband, Boolean fieldRelative, SwerveChassisSubsystem chassis) {
    
    // Allows for these objects to be refrenced class wide
    this.chassis = chassis;
    this.useDeadband = useDeadband;
    this.fieldRelative = fieldRelative;

    // Configure the rotarPID for continuous input
    rotaryPID.enableContinuousInput(0, 360);

    // Config aimBot for continuous input.
    MiscConstants.aimBot.enableContinuousInput(-180, 180);

    // Theres no movement without a drivetrain
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get robot's current angle
    double robotCurrentAngle = chassis.getGyroZ().getDegrees() % 360; // This could be broken depending on if the direction is CW or CCW

    // Resets to 360 when the angle hits 0
    if (robotCurrentAngle < 0) {
      robotCurrentAngle += 360;
    }

    // Obtain the singleton controller instance (HID object) (Type must be stellar controller)
    ControllerIO cIO = ControllerIO.getPrimaryInstance();
    StellarController driveController = cIO.stellarController.getHID();


    // (Temporary) Get raw controller axis
    double xSpeed = driveController.getLeftX() * -1; // Invert X Axis
    double ySpeed = driveController.getLeftY();
    Rotation2d rotaryAngle = driveController.getRightRotary();
    double rot; // Value will be assigned based off if were using vision or not

    rot = rotaryPID.calculate(robotCurrentAngle, rotaryAngle.getDegrees());
    SmartDashboard.putString("RotationStatus", "ControllerControlled");

    if (useDeadband) { // Apply deadband if specified
      double[] deadbandFilter = MiscUtils.circularDeadband(xSpeed, ySpeed, IOConstants.kDriveDeadband);

      xSpeed = deadbandFilter[0];
      ySpeed = deadbandFilter[1];

    }

    // Bind a button to reseting the gyro to the odometry.
    if (driveController.getBButtonPressed()) {
      chassis.zeroHeading();
    }

    // Define the variables for holding the modified values in a broader scope
    double xSpeedCommanded;
    double ySpeedCommanded;

    // Set final speeds
    xSpeedCommanded = xSpeed;
    ySpeedCommanded = ySpeed;
    m_currentRotation = rot;

    // Convert the commanded speeds into the correct units for the drivetrain
    double dashTranslationSpeed = SmartDashboard.getNumber("TranslationSpeed", DriveConstants.kMaxSpeedMetersPerSecond);
    double dashAngularSpeed =  SmartDashboard.getNumber("RotationSpeed", DriveConstants.kMaxAngularSpeedFactor);

    double xSpeedDelivered = xSpeedCommanded * dashTranslationSpeed * (BaseConstants.DriveConstants.elevatorSpeedOverride ? 0.3 : 1);
    double ySpeedDelivered = ySpeedCommanded * dashTranslationSpeed * (BaseConstants.DriveConstants.elevatorSpeedOverride ? 0.3 : 1);
    double rotDelivered = m_currentRotation * dashAngularSpeed * (BaseConstants.DriveConstants.elevatorSpeedOverride ? 0.3 : 1);

    // Convert values into either a robot relative or field oriented Chassis Speed object
    ChassisSpeeds positionCommanded = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, chassis.getGyroZ())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    // Apply the new ChassisSpeed object to the drivetrain
    chassis.drive(positionCommanded);

    // (Temporary) // Calls to update telemetry on SmartDashboard
    chassis.getChassisSpeeds();
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
