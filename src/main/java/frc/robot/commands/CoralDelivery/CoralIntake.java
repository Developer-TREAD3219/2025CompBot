package frc.robot.commands.CoralDelivery;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.coralDeliveryConstants;
import frc.robot.subsystems.CoralDeliverySubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.coralDeliveryConstants;

public class CoralIntake extends Command {

    CoralDeliverySubsystem coralDeliverySubsystem;
    DigitalInput CoralInPLaceSensor = new DigitalInput(coralDeliveryConstants.kCoralInPlaceID);
    DigitalInput Coral= new DigitalInput(coralDeliveryConstants.kCoralInPlaceID);


    public CoralIntake(CoralDeliverySubsystem coralDeliverySubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.coralDeliverySubsystem = coralDeliverySubsystem;
        addRequirements(coralDeliverySubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        coralDeliverySubsystem.intake();
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

    // Returns true when the command should end.
   // @Override
    //public boolean isFinished() {
        //TODO: need to set kCoralInPlaceID boolean to true when coral is in place
    //    return CoralDeliverySubsystem.CoralInPlace();
    //}
    

   
    //public boolean CoralInPlace() {
        //returns 1 if beam is broken or 0 if beam is not broken
       // return CoralInPLaceSensor.get();
      //}
}


