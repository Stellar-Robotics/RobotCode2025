// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotChassis.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.RobotContainer;
import frc.robot.BaseConstants.DriveConstants;
import frc.robot.RobotUtilities.MiscUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveChassisSubsystem extends SubsystemBase {
  
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      false);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      false);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      false);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      false);

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  // NavX gyro sensor
  private final AHRS m_navxgyro = new AHRS(NavXComType.kMXP_SPI);

  // Pose estimator object
  public SwerveDrivePoseEstimator swervePoseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kDriveKinematics, 
    Rotation2d.fromDegrees(-m_navxgyro.getAngle()  + 90), 
    getModulePositions(),
    new Pose2d(15.380, 0.468, getGyroZ())
  );

  // Create a field object for pose visualization
  private Field2d field;

  /** Creates a new DriveSubsystem. */
  public SwerveChassisSubsystem() {

    // Define field object and publish it
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // AutoBuilder.configure(this::getPose,
    //   this::resetOdometry,
    //   this::getChassisSpeeds,
    //   (speeds) -> drive(speeds),
    //   new PPHolonomicDriveController
    //     (
    //       new PIDConstants(5.0, 0.0, 0.0),
    //       new PIDConstants(5.0, 0.0, 0.0)
    //     ),
    //   Configs.PathPlanner.pathPlannerConfig,
    //   MiscUtils.isRedAlliance(),
    //   this
    // );

    // Zero Robot Heading
    this.zeroHeading();
  }

  @Override
  public void periodic() {


    // Update pose estimation
    swervePoseEstimator.updateWithTime(
      Timer.getFPGATimestamp(), 
      Rotation2d.fromDegrees(-m_navxgyro.getAngle()), 
      getModulePositions());

    // Add the (Reef) vision estimate if new data is available
    var visionEstReef = RobotContainer.getSingletonInstance().getVisionEstimate(true);
    if (visionEstReef != null) {
      // add vision estimate to pose
      SmartDashboard.putBoolean("VisionEstimateStatusReef", true);
      swervePoseEstimator.addVisionMeasurement(visionEstReef, Timer.getFPGATimestamp());
    } else {
      SmartDashboard.putBoolean("VisionEstimateStatusReef", false);

      // Add the (General) vision estimate if new data is availible
      var visionEstGeneral = RobotContainer.getSingletonInstance().getVisionEstimate(false);
      if (visionEstGeneral != null) {
        // add vision estimate to pose
        SmartDashboard.putBoolean("VisionEstimateStatusGeneral", true);
        swervePoseEstimator.addVisionMeasurement(visionEstGeneral, Timer.getFPGATimestamp());
      } else {
        SmartDashboard.putBoolean("VisionEstimateStatusGeneral", false);
      }

    }

    // Update the field object with the odometry data
    field.setRobotPose(getPose());
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {field.setRobotPose(pose);});
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {field.getObject("target pose").setPose(pose);});
    PathPlannerLogging.setLogActivePathCallback((poses) -> {field.getObject("path").setPoses(poses);});

    SmartDashboard.putNumber("Gyro Angle Yaw", -m_navxgyro.getAngle());
    
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    swervePoseEstimator.resetPosition(
      Rotation2d.fromDegrees(-m_navxgyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      pose);
  }

  public void initAutoBuilder() {
    AutoBuilder.configure(this::getPose,
      this::resetOdometry,
      this::getChassisSpeeds,
      (speeds) -> drive(speeds),
      new PPHolonomicDriveController
        (
          new PIDConstants(5.0, 0.0, 0.0),
          new PIDConstants(5.0, 0.0, 0.0)
        ),
      Configs.PathPlanner.pathPlannerConfig,
      MiscUtils.isRedAlliance(),
      this
    );
  }

  // The primary method of commanding the chassis speeds.  Any higher control leveles should be handled outside this class.
  public void drive(ChassisSpeeds speeds) {

    // Create module states from the chassis speeds
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    // Desaturate to make sure all wheels can handle the load
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Apply the individule swerve module states
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // m_navxgyro.reset();
    // m_navxgyro.zeroYaw();
    m_navxgyro.reset();
    // m_navxgyro.setGyroAngleZ(DriveConstants.kGyroOffset); // Add offset so the intake is the front
    m_navxgyro.setAngleAdjustment(DriveConstants.kGyroOffset);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_navxgyro.getAngle()).getDegrees();
  }

  // Need to get the Rotation2D object from gyro for certain field oriented control commands
  public Rotation2d getGyroZ() {
    return Rotation2d.fromDegrees(-m_navxgyro.getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_navxgyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public ChassisSpeeds getChassisSpeeds() {

    SwerveModuleState[] currentModuleStates = new SwerveModuleState[4];

    currentModuleStates[0] = m_frontLeft.getState();
    currentModuleStates[1] = m_frontRight.getState();
    currentModuleStates[2] = m_rearLeft.getState();
    currentModuleStates[3] = m_rearRight.getState();

    SmartDashboard.putNumber("Mod-FL", currentModuleStates[0].angle.getDegrees());
    SmartDashboard.putNumber("Mod-FR", currentModuleStates[1].angle.getDegrees());
    SmartDashboard.putNumber("Mod-RL", currentModuleStates[2].angle.getDegrees());
    SmartDashboard.putNumber("Mod-RR", currentModuleStates[3].angle.getDegrees());

    ChassisSpeeds currentChassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(currentModuleStates);

    return currentChassisSpeeds;
    
  }

}
