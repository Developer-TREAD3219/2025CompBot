package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class goToElevatorL2 extends Command {
    private final ElevatorSubsystem elevatorSubsystem;

    public goToElevatorL2(ElevatorSubsystem subsystem) {
        elevatorSubsystem = subsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.goToElevatorL2();
        System.out.println("Elevator L2");
    }

    @Override
    public void execute() {
        // Placeholder for execution code
    }

    @Override
    public boolean isFinished() {
        // Placeholder for completion condition
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // Placeholder for cleanup code
    }
}