// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.CoralMech.Subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotUtilities.MiscUtils;

public class CoralMech extends SubsystemBase {

  // Declare the variables to hold our motor controller and closed loop controller objects.
  private final SparkFlex rollerMotor;
  private final SparkClosedLoopController rollerCLC;

  // Declare the variables to hold the extention motor
  private final SparkMax coralExtension;
  private final SparkClosedLoopController extensionCLC;
  private final RelativeEncoder extensionEncoder;

  private final SparkMax coralExtension2;

  public CoralMech() {

    // Create a new motor controller object and store it in the rollerMotor variable.
    rollerMotor = new SparkFlex(MechanismConstants.CoralMechValues.rollerMotorID, MotorType.kBrushless);

    coralExtension = new SparkMax(MechanismConstants.CoralMechValues.extensionMotorID, MotorType.kBrushless);

    coralExtension2 = new SparkMax(MechanismConstants.CoralMechValues.extensionMotorID2, MotorType.kBrushless);



    // Get the closed loop controller object from the newly created motor controller object that is stored in
    // the rollerMotor variable.  We'll then create a refrence to that closed loop controller object in the
    // rollerCLC variable.
    rollerCLC = rollerMotor.getClosedLoopController();

    extensionCLC = coralExtension.getClosedLoopController();

    extensionEncoder = coralExtension.getEncoder();

    // Call the 'configure' method inside the motor controller object (stored in the rollerMotor
    // variable).  We'll then pass in the configuration parameters that the 'configure' method is looking for.
    rollerMotor.configure(Configs.CoralMechConfig.rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralExtension.configure(Configs.CoralMechConfig.extensionMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralExtension2.configure(Configs.CoralMechConfig.extensionMotorConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setVelocity(double velocity) {

    // Call the 'setRefrence' method inside the closed loop controller object
    // that we are refrencing in the rollerCLC variable.  We'll pass into the method
    // the velocity variable and the mode we want it to use.
    rollerCLC.setReference(velocity, ControlType.kVelocity);

  }

public void setRollerPower(double setPower) {

    rollerMotor.set(setPower);

}

  public void goToPosition(double postition) {
    double clampedPosition = MiscUtils.clamp(MechanismConstants.CoralMechValues.minExtension, MechanismConstants.CoralMechValues.maxExtension, postition);
    REVLibError error = extensionCLC.setReference(clampedPosition, ControlType.kPosition);
    if (error != REVLibError.kOk) {
      coralExtension.stopMotor();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CoralExtension", extensionEncoder.getPosition());
  }
}
