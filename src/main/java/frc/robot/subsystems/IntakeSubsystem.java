// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.coralDeliveryConstants;
// import frc.robot.Constants.coralDeliveryConstants;

// public class IntakeSubsystem extends SubsystemBase{

//     // this may not be used if we are not using a servo to open the intake door
//     //TODO: this was throwing an error complaining about the ID's being assigned multiple times. I agree this shouldn't matter because one is DIO and the other is PWM. Need to investigate. set at 98 and 99 for the mean time
//     Servo intakeServo = new Servo(coralDeliveryConstants.kIntakeServoID);

//     public IntakeSubsystem() {
//         // Code for opening the door flaps for intake
//     }
//     public void openServo() {
//         // Set the servo to the open position (e.g., 180 degrees)
//         intakeServo.setAngle(coralDeliveryConstants.kIntakeServoOpen);
//         }

//     public void closeServo() {
//         // Set the servo to the open position (e.g., 180 degrees)
//         intakeServo.setAngle(coralDeliveryConstants.kIntakeServoClosed);
//         }
// }
