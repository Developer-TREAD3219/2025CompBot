package frc.robot.commands;

import frc.robot.subsystems.LimeLightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ReefAlignment extends Command {
    LimeLightSubsystem m_LimeLightSubsystem;
    double m_Yaw, m_Skew;

    public ReefAlignment() {

    }

    public void leftAlignment() {
        m_Yaw = m_LimeLightSubsystem.getYaw();
        m_Skew = m_LimeLightSubsystem.getSkew();
        System.out.println("left Alignment, m_Yaw = " + m_Yaw + ", Skew = " + m_Skew);
        
    }
}

