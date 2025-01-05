// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotChassis.Commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BaseConstants.DriveConstants;
import frc.robot.BaseConstants.IOConstants;
import frc.robot.RobotChassis.Subsystems.SwerveChassisSubsystem;
import frc.robot.RobotControl.ControllerIO;
import frc.robot.RobotUtilities.MiscUtils;
import frc.robot.RobotUtilities.SwerveUtils;

public class DriveWithRotaryCommand extends Command {
  
  // Create variables for the arguments passed in so they can be refrenced class wide
  private SwerveChassisSubsystem chassis;
  private boolean rateLimit;
  private boolean useDeadband;
  private boolean fieldRelative;

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Define the PID for the rotary dial
  private PIDController rotaryPID = new PIDController(0.01, 0 , 0);

    /**
   * Method to drive the robot using joystick info.
   *
   * @param chassis       The SubsystemContainer object the output is applied to.
   * @param useDeadband   Whether to utiltize deadband for the controls.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */

  public DriveWithRotaryCommand(Boolean rateLimit, Boolean useDeadband, Boolean fieldRelative, SwerveChassisSubsystem chassis) {

    // Allows for these objects to be refrenced class wide
    this.chassis = chassis;
    this.rateLimit = rateLimit;
    this.useDeadband = useDeadband;
    this.fieldRelative = fieldRelative;

    // Configure the rotarPID for continuous input
    rotaryPID.enableContinuousInput(0, 360);

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

    // Obtain the singleton controller instance
    ControllerIO cIO = ControllerIO.getPrimaryInstance(ControllerIO.controllerType.STELLAR);

    // (Temporary) Get raw controller axis
    double xSpeed = cIO.stellarController.getLeftX() * -1; // Invert X Axis
    double ySpeed = cIO.stellarController.getLeftY();
    Rotation2d rotaryAngle = cIO.stellarController.getRightRotary();

    // Run the absolute rotary angle through the PID controller
    double rot = rotaryPID.calculate(robotCurrentAngle, rotaryAngle.getDegrees());

    if (useDeadband) { // Apply deadband if specified
      double[] deadbandFilter = MiscUtils.circularDeadband(xSpeed, ySpeed, IOConstants.kDriveDeadband);

      xSpeed = deadbandFilter[0];
      ySpeed = deadbandFilter[1];
      
    }

    // Define the variables for holding the modified values in a broader scope
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // Some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // Some small number to avoid floating-point errors with equality checking
          // Keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }

      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

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
