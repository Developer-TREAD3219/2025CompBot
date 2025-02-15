package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;
import com.fasterxml.jackson.databind.type.PlaceholderForType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.coralDeliveryConstants;
import edu.wpi.first.wpilibj.DigitalInput;

    // :  CoralDeliverySubsystem should extend from SubsystemBase. See an example subsystem here https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html
    // :  Make placeholder intake and outtake methods for the coral delivery subsystem

public class CoralDeliverySubsystem extends SubsystemBase {
   
    SparkMax coralDeliveryMotor = new SparkMax(coralDeliveryConstants.kCoralDeliveryMotorID, MotorType.kBrushless);
    RelativeEncoder coralEncoder = coralDeliveryMotor.getEncoder();
    DigitalInput CoralInPlaceSensor = new DigitalInput(coralDeliveryConstants.kCoralInPlaceID);
    DigitalInput CoralInElevatorSensor = new DigitalInput(coralDeliveryConstants.kCoralInElevatorID);

    public CoralDeliverySubsystem() {
        coralDeliveryMotor.set(coralDeliveryConstants.kIntakeSpeed); // the % output of the motor, between -1 and 1
    }

    // Placeholder for intake
    public void intake() {
        // Code for intake
        coralDeliveryMotor.set(coralDeliveryConstants.kIntakeSpeed); // the % output of the motor, between -1 and 1
        //         elevator shouldn't move when coral has broken the beam, but not in place
        //         there will be a beam break sensor that will detect when coral has entered, 
        //             a second sensor will tell us when the coral is captured)
        //             Smartdashboard button to enable/disable intake sensor
    }

    // Placeholder for outtake
    public void outtake() {
        // Code for outtake
        coralDeliveryMotor.set(coralDeliveryConstants.kOuttakeSpeed); // the % output of the motor, between -1 and 1
    }

    
    public void stopMotor() {
        // Stop the motor by setting its speed to 0.0
        coralDeliveryMotor.set(0.0);
     }

    public void manualSpin(double speed) {
        // Spin the motor at a given speed
        coralDeliveryMotor.set(speed);
        System.out.println("Motor Speed: " + speed);
    }

     public boolean CoralInElevator() {
        //returns 1 if beam is broken or 0 if beam is not broken
        return CoralInElevatorSensor.get();
     }
        
    public boolean CoralInPlace() {
        //returns 1 if beam is broken or 0 if beam is not broken
        return CoralInPlaceSensor.get();
      }

    //      //placeholder for possible periodic for Autonomous
    // public void periodic(){
    //     // This method will be called once per scheduler run
    // }
}
