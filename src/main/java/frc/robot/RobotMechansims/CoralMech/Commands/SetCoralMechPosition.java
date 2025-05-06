// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotMechansims.CoralMech.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.MechanismConstants;
import frc.robot.RobotMechansims.CoralMech.Subsystems.CoralMech;
import frc.robot.RobotMechansims.MechanismConstants.CoralMechValues.CORALEXTENSIONPOSITION;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralMechPosition extends Command {

  private CoralMech subsystem;
  private double position;
  private boolean reset;

  /** Creates a new ToggleCoralExtension. */
  public SetCoralMechPosition(CoralMech mechanism, double pos, boolean reset) {
    // Use addRequirements() here to declare subsystem dependencies.
    subsystem = mechanism;
    position = pos;
    this.reset = reset;
    addRequirements(mechanism);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    subsystem.goToPosition(position);

    if (reset) {
      MechanismConstants.CoralMechValues.currentPos = CORALEXTENSIONPOSITION.MIDDLE;
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
