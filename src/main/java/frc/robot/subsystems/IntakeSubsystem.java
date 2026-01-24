// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  private boolean m_intakeExtended = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public void toggleIntakeExtentions() {
    // Code to toggle the intake mechanism
    if (m_intakeExtended == true) {
          System.out.println("[INTAKE] Retract intake");
          m_intakeExtended = false;
    } else {
          System.out.println("[INTAKE] Extend intake");
          m_intakeExtended = true;
    }
  }

public void startIntakeRollers(){
  //Starts the Intake rollers to intake fuel
  System.out.println("[INTAKE] Rollers started");
}

public void stopIntakeRollers(){
  //Stops the Intake rollers
  System.out.println("[INTAKE] Rollers stopped");
}

public void reverseIntakeRollers(){
  //Reverses the Intake rollers to get stuff unstuck
  System.out.println("[INTAKE] Rollers reversed");
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
