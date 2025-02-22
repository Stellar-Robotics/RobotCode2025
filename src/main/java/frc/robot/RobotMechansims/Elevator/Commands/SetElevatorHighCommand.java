package frc.robot.RobotMechansims.Elevator.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.Elevator.Subsystems.Elevator;

public class SetElevatorHighCommand extends Command {

    private Elevator elevator;

    public SetElevatorHighCommand(Elevator elev) {
        this.elevator = elev;
    }

    @Override
    public void execute() {
        this.elevator.goToPositionClamped(178);
    }
} 
