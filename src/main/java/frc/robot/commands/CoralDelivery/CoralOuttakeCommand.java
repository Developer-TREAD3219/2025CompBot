package frc.robot.commands.CoralDelivery;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDeliverySubsystem;

public class CoralOuttakeCommand extends Command {

    CoralDeliverySubsystem coralDeliverySubsystem;

    public CoralOuttakeCommand(CoralDeliverySubsystem coralDeliverySubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.coralDeliverySubsystem = coralDeliverySubsystem;
        addRequirements(coralDeliverySubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        coralDeliverySubsystem.outtake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralDeliverySubsystem.stopMotor();
    }
//instead of CoralInPlace() we could use CoralInElevator() if we want to check if the coral is in the elevator
//if coral is scored in the elevator, the command will end
// invert motors to outtake
    // Returns true when the command should end.  This is when the coral has broken the beam for "in place".
    @Override
    public boolean isFinished() {
        return false;
    }   
}



