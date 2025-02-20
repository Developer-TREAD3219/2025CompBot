package frc.robot.commands.CoralDelivery;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.coralDeliveryConstants;
import frc.robot.subsystems.CoralDeliverySubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.RumbleHelper;

public class CoralIntakeCommand extends Command {

    CoralDeliverySubsystem coralDeliverySubsystem;
    DigitalInput m_deliverySensor; 
    Boolean currentSensorState = false;
    Boolean previousSensorState = false;
    XboxController m_driverController;
    RumbleHelper rumble;
    private boolean isFinished = false;
    enum Stage { STAGE0, STAGE1, STAGE2 }
    Stage m_stage = Stage.STAGE0;
    


    public CoralIntakeCommand(CoralDeliverySubsystem coralSubsystem, XboxController driverController) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.m_deliverySensor = coralSubsystem.getCoralInPlaceSensor();
        this.coralDeliverySubsystem = coralSubsystem;
        this.m_driverController = driverController;
        this.isFinished = false;
        addRequirements(coralDeliverySubsystem);
        this.rumble = new RumbleHelper(driverController);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        coralDeliverySubsystem.spinMotor(coralDeliveryConstants.kIntakeSpeedStage1);
        //get the current state of the input sensor
        previousSensorState = !m_deliverySensor.get();
        // set the state to 1
        m_stage = Stage.STAGE1;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // check the sensor before running the logicf
        boolean currentSensorState = m_deliverySensor.get();
        // use the current stage to determine the correct action
        switch (m_stage) {
            // STAGE1 we should leave this stage once the sensor reads high
            case STAGE1:
                if (currentSensorState) {
                    coralDeliverySubsystem.spinMotor(coralDeliveryConstants.kIntakeSpeedStage2);
                    m_stage = Stage.STAGE2;
                }
                break;
            // STAGE2 we should leave this stage once the sensor reads low
            case STAGE2:
                if (!currentSensorState) {
                    m_stage = Stage.STAGE0;
                    isFinished = true;
                }
                break;

        }
        // update the previous state once we are done thinking
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
        return isFinished;
    }
}

