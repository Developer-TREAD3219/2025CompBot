package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
    // :  CoralDeliverySubsystem should extend from SubsystemBase. See an example subsystem here https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html
    // :  Make placeholder intake and outtake methods for the coral delivery subsystem
    // TODO: It sounds like this is going to be run with PWM. we should learn how that works
public class CoralDeliverySubsystem extends SubsystemBase {
   
    public CoralDeliverySubsystem() {
        Spark spark = new Spark(0); // 0 is the RIO PWM port this is connected to

        spark.set(-0.75); // the % output of the motor, between -1 and 1
    }

    // Placeholder for intake
    public void intake() {
        // Code for intake
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
