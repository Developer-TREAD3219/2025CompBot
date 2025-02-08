package frc.robot.subsystems;
import java.lang.ModuleLayer.Controller;

import com.fasterxml.jackson.databind.type.PlaceholderForType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.coralDeliveryConstants;
    // :  CoralDeliverySubsystem should extend from SubsystemBase. See an example subsystem here https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html
    // :  Make placeholder intake and outtake methods for the coral delivery subsystem
    // TODO: It sounds like this is going to be run with PWM. we should learn how that works
public class CoralDeliverySubsystem extends SubsystemBase {
   
    public CoralDeliverySubsystem() {
        Spark spark = new Spark(coralDeliveryConstants.kCoralDeliveryMotorID); // 0 is the RIO PWM port this is connected to

        spark.set(-0.75); // the % output of the motor, between -1 and 1
    }

    // Placeholder for intake
    public void intake() {
        // Code for intake

        // TODO: intake shuts off motor when coral is in Place and rumble driver (()Controller

        //         elevator shouldn't move when coral has broken the beam, but not in place
        //         there will be a beam break sensor that will detect when coral has entered, 
        //             a second sensor will tell us when the coral is captured)
        //             Smartdashboard button to enable/disable intake sensor
    }

    // Placeholder for outtake
    public void outtake() {
        // Code for outtake
    }
    //placeholder for possible periodic for Autonomous
    public void periodic(){
        // This method will be called once per scheduler run
    }
   
}
