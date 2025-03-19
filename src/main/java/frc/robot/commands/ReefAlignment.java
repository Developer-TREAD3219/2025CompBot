package frc.robot.commands;

import frc.robot.subsystems.LimeLightSubsystem;

import java.lang.reflect.Array;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ReefAlignment extends Command {
    LimeLightSubsystem m_LimeLightSubsystem;
    double m_Yaw, m_Skew;
    DriveSubsystem m_DriveSubsystem;
    ElevatorSubsystem m_ElevatorSubsystem;
    boolean alignedStage1;

    public ReefAlignment(LimeLightSubsystem limelight, DriveSubsystem drive, ElevatorSubsystem elevator) {
        m_LimeLightSubsystem = limelight;
        m_DriveSubsystem = drive;
        m_ElevatorSubsystem = elevator;

    }

    public void leftAlignment() {
        m_Yaw = m_LimeLightSubsystem.getYaw();
        m_Skew = m_LimeLightSubsystem.getSkew();
        System.out.println("left Alignment, m_Yaw = " + m_Yaw + ", Skew = " + m_Skew);
        
    }

    @Override
    public void initialize(){
        alignedStage1 = false;
    }

    @Override
    public void execute(){
        double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        double xPosition = botpose[0];
        double yPosition = botpose[2];
        double yaw = botpose[4];
        double targetSeen = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double rotAdjust = 0;
        double yAdjust = 0;
        double xAdjust = 0;
        System.out.println("X  : " + xPosition);
        System.out.println("Y  : " + yPosition);
        System.out.println("Yaw: " + yaw);

        if (yaw > 2.5){
            rotAdjust = -0.05;
        }
        if (yaw < -2.5){
            rotAdjust = 0.05;
        }
        // left allignment
        if (xPosition > -.17){
            yAdjust = 0.05;
        }
        if (xPosition < -.19){
            yAdjust = -0.05;
        }
        // left allignment
        if (yPosition > .7){
            xAdjust = -0.05;
        }
        if (yPosition < .6){
            xAdjust = 0.05;
        }
        // drive to pose code
        if (!alignedStage1 && targetSeen == 1 && Math.abs(xAdjust)+Math.abs(yAdjust)+Math.abs(rotAdjust) != 0){
        m_DriveSubsystem.drive(xAdjust, yAdjust, rotAdjust, false);
        }
        if (!alignedStage1 && Math.abs(xAdjust)+Math.abs(yAdjust)+Math.abs(rotAdjust) ==0){
            alignedStage1 = true;
        }
        if (alignedStage1){
            m_DriveSubsystem.drive(-0.1, 0, 0, false);
        }
    }
}

