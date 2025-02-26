// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.CoralMech.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotMechansims.CoralMech.Subsystems.CoralMech;
import frc.robot.RobotMechansims.MechanismConstants.CoralMechValues.CORALEXTENSIONPOSITION;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IncramentCoralExtensionCommand extends Command {

  private CoralMech coral;
  private boolean forward;
  /** Creates a new IncramentCoralExtensionCommand. */
  public IncramentCoralExtensionCommand(CoralMech coral, boolean goingForward) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coral = coral;
    forward = goingForward;

    addRequirements(coral);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (forward) {
      // Going Forward
      if (MechanismConstants.CoralMechValues.currentPos == CORALEXTENSIONPOSITION.MIDDLE) {
        coral.goToPosition(43); // TBD FORWARD POS
        MechanismConstants.CoralMechValues.currentPos = CORALEXTENSIONPOSITION.FORWARD;
        SmartDashboard.putString("Going To", "Forward");
      }

      if (MechanismConstants.CoralMechValues.currentPos == CORALEXTENSIONPOSITION.BACK) {
        coral.goToPosition(0); // Set
        MechanismConstants.CoralMechValues.currentPos = CORALEXTENSIONPOSITION.MIDDLE;
        SmartDashboard.putString("Going To", "Middle");
      }

    } else {
      // Going Backward
      if (MechanismConstants.CoralMechValues.currentPos == CORALEXTENSIONPOSITION.MIDDLE) {
        coral.goToPosition(-40); // TBD BACK POS
        MechanismConstants.CoralMechValues.currentPos = CORALEXTENSIONPOSITION.BACK;
        SmartDashboard.putString("Going To", "Back");
      }
      
      if (MechanismConstants.CoralMechValues.currentPos == CORALEXTENSIONPOSITION.FORWARD) {
        coral.goToPosition(0); // Set
        MechanismConstants.CoralMechValues.currentPos = CORALEXTENSIONPOSITION.MIDDLE;
        SmartDashboard.putString("Going To", "Middle");
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
