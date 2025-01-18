package frc.robot.RobotMechansims.PrimaryElevator.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.PrimaryElevator.Subsystems.PrimaryElevator;

public class SetElevatorCommand extends Command {

    private PrimaryElevator elevator;
    private double position;

    public SetElevatorCommand(PrimaryElevator elev, double pos) {
        this.elevator = elev;
        this.position = pos;
    }

    @Override
    public void execute() {
        this.elevator.goToPositionClamped(position);
    }
} 
