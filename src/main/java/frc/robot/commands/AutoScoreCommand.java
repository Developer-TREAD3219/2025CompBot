package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class AutoScoreCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final XboxController gunnerController;

    public AutoScoreCommand(ElevatorSubsystem elevatorSubsystem, XboxController gunnerController) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.gunnerController = gunnerController;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        String commandStatus = "Waiting for command";

        // Add shift to the Left when the Left Trigger is pressed
        if (gunnerController.getLeftTriggerAxis() > 0.9) {
            if (gunnerController.getYButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL1);
                commandStatus = "Success: Elevator can score L1";
            } else if (gunnerController.getXButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL2);
                commandStatus = "Success: Elevator can score L2";
            } else if (gunnerController.getBButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL3);
                commandStatus = "Success: Elevator can score L3";
            } else if (gunnerController.getAButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL4);
                commandStatus = "Success: Elevator can score L4";
            }
        }

        // Add shift to the Right when the Right Trigger is pressed
        if (gunnerController.getRightTriggerAxis() > 0.9) {
            if (gunnerController.getAButton()) {
                System.out.println("A Button Pressed");
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL1);
                commandStatus = "Success: Elevator can score L1";
            } else if (gunnerController.getYButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL2);
                commandStatus = "Success: Elevator can score L2";
            } else if (gunnerController.getXButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL3);
                commandStatus = "Success: Elevator can score L3";
            } else if (gunnerController.getBButton()) {
                // elevatorSubsystem.setPositionInches(ElevatorConstants.kL4);
                commandStatus = "Success: Elevator can score L4";
            }
        }

        // Print the latest command status
        System.out.println(commandStatus);
    }

    @Override
    public void execute() {
        // Implementation of execute method
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}