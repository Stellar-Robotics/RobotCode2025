package frc.robot.RobotMechansims.ClimbMech.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.ClimbMech.Subsystems.ClimbSubsystem;

public class ClimberGoToPositionCommand extends Command {
    ClimbSubsystem subsystem;
    double setPoint;

    public ClimberGoToPositionCommand(ClimbSubsystem sub, double point) {
        subsystem = sub;
        setPoint = point;
    }

    @Override
    public void execute() {
        subsystem.goTP(setPoint);
    }

}
