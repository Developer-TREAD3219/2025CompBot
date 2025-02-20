package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.coralDeliveryConstants;

import edu.wpi.first.wpilibj.DigitalInput; 


public class CoralDeliverySubsystem extends SubsystemBase {
   
    Spark coralDeliveryMotor = new Spark(coralDeliveryConstants.kCoralDeliveryMotorID);
    DigitalInput CoralInPlaceSensor = new DigitalInput(coralDeliveryConstants.kCoralInPlaceID);
    // DigitalInput CoralInElevatorSensor = new DigitalInput(coralDeliveryConstants.kCoralInElevatorID);

    // Bool traking weather the system believes the coral is in scoring position
    private boolean coralInScoringPosition = false;

    public CoralDeliverySubsystem() {
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

    public void spinMotor(double speed) {
        // Spin the motor at a given speed
        System.out.println("attempting spin at" +speed);
        coralDeliveryMotor.set(speed);
    }
    public void manualSpin(double speed) {
        // Spin the motor at a given speed
        coralDeliveryMotor.set(speed);
        System.out.println("Motor Speed: " + speed);
    }
    public DigitalInput getCoralInPlaceSensor(){
        return CoralInPlaceSensor;
    }

    //  public boolean CoralInElevator() {
    //     //returns 1 if beam is broken or 0 if beam is not broken
    //     return CoralInElevatorSensor.get();
    // }
        
    // public boolean CoralInPlace() {
    //     //returns 1 if beam is broken or 0 if beam is not broken
    //     return CoralInPlaceSensor.get();
    //   }

    public boolean isCoralInScoringPosition() {
        return coralInScoringPosition;
    }

    public void setCoralInScoringPosition(boolean coralInScoringPosition) {
        this.coralInScoringPosition = coralInScoringPosition;
    }
    
    @Override
    public void periodic() {
    }

}
