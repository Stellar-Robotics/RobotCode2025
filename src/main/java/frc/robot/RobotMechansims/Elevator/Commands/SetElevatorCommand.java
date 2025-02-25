package frc.robot.RobotMechansims.Elevator.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechansims.Elevator.Subsystems.Elevator;
import frc.robot.RobotMechansims.Elevator.Subsystems.Elevator.POSITIONS;

public class SetElevatorCommand extends Command {

    private Elevator elevator;
    private POSITIONS position;

    public SetElevatorCommand(Elevator elev, POSITIONS position) {
        this.elevator = elev;
        this.position = position;
        addRequirements(elev);
    }

    @Override
    public void execute() {

        switch (position) {
            case LOW:
                this.elevator.goToPositionClamped(0);
                break;
            case MID:
                this.elevator.goToPositionClamped(80);
                break; 
            case HIGH:
                this.elevator.goToPositionClamped(178);
            default:
                break;
        }

        System.out.println("Elevator " + position);

    }
} 
