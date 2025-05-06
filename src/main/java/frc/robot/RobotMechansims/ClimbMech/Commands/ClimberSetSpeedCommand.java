package frc.robot.RobotMechansims.ClimbMech.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.ClimbMech.Subsystems.ClimbSubsystem;

public class ClimberSetSpeedCommand extends Command {
    ClimbSubsystem subsystem;
    double setSpeed;

    public ClimberSetSpeedCommand(ClimbSubsystem sub, double speed) {
        subsystem = sub;
        setSpeed = speed;
    }

    @Override
    public void execute() {
        subsystem.setPosition(setSpeed);
    }

    @Override
    public boolean isFinished() {
      return true;
    }

}
