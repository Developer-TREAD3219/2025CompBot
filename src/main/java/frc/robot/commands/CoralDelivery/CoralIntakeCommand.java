package frc.robot.commands.CoralDelivery;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.coralDeliveryConstants;
import frc.robot.subsystems.CoralDeliverySubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.coralDeliveryConstants;
import frc.robot.utils.RumbleHelper;

public class CoralIntakeCommand extends Command {

    CoralDeliverySubsystem coralDeliverySubsystem;
    DigitalInput m_deliverySensor; 
    Boolean currentSensorState = false;
    Boolean previousSensorState = false;
    XboxController m_driverController;
    RumbleHelper rumble;

    public CoralIntakeCommand(CoralDeliverySubsystem coralSubsystem, XboxController driverController) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_deliverySensor = coralSubsystem.getCoralInPlaceSensor();
        System.out.println("BERRERROYYOUBOUH"+ coralSubsystem);
        this.coralDeliverySubsystem = coralSubsystem;
        this.m_driverController = driverController;
        addRequirements(coralDeliverySubsystem);
        RumbleHelper rumble = new RumbleHelper(driverController);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        coralDeliverySubsystem.intake();
        //get the current state of the input sensor
        previousSensorState = m_deliverySensor.get();
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
        rumble.rumbleForDuration(0.3, 1, 0.5);
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return coralDeliverySubsystem.CoralInPlace();
    }
}


