package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    // this may not be used if we are not using a servo to open the intake door

    Servo intakeServo = new Servo(IntakeConstants.kIntakeServoID);

    public IntakeSubsystem() {
        // Code for opening the door flaps for intake

    }
    public void openServo() {
        // Set the servo to the open position (e.g., 180 degrees)
        intakeServo.setAngle(IntakeConstants.kIntakeServoOpen);
        }

    public void closeServo() {
        // Set the servo to the open position (e.g., 180 degrees)
        intakeServo.setAngle(IntakeConstants.kIntakeServoClosed);
        }
    

}
