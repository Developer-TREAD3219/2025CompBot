package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    Servo IntakeServo = new Servo(IntakeConstants.kIntakeServoMotorID);

    public IntakeSubsystem() {
        // Code for opening the door flaps for intake
    }
    public void openServo() {
        // Move servo to 90 degrees.
        IntakeServo.setPosition(1);
    }
    public void closeServo() {
        // Move servo to 0 degrees.
        IntakeServo.setPosition(0);
    }
}
