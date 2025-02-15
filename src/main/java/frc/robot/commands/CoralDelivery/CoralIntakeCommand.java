package frc.robot.commands.CoralDelivery;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.coralDeliveryConstants;
import frc.robot.subsystems.CoralDeliverySubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.coralDeliveryConstants;

public class CoralIntakeCommand extends Command {

    CoralDeliverySubsystem coralDeliverySubsystem;
    DigitalInput m_deliverySensor = new DigitalInput(coralDeliveryConstants.kCoralInPlaceID);
    DigitalInput Coral= new DigitalInput(coralDeliveryConstants.kCoralInPlaceID);
    Boolean currentSensorState = false;
    Boolean previousSensorState = false;

    public CoralIntakeCommand(CoralDeliverySubsystem coralDeliverySubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.coralDeliverySubsystem = coralDeliverySubsystem;
        addRequirements(coralDeliverySubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        coralDeliverySubsystem.intake();
        //get the current state of the input sensor
        boolean previousSensorState = m_deliverySensor.get();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean currentSensorState = m_deliverySensor.get();
        if (previousSensorState && !currentSensorState){
            this.end(true);
        }
        previousSensorState = currentSensorState;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralDeliverySubsystem.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //TODO: need to set kCoralInPlaceID boolean to true when coral is in place
        return coralDeliverySubsystem.CoralInPlace();
    }
}


