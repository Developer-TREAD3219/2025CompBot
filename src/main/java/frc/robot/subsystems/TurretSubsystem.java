// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
  }

  public void setAllianceZoneLock() {
    // Code to set turret to alliance zone lock position
    System.out.println("[TURRET] Set to Alliance Zone Lock position");
  }
  
  public void lockOntoHub(){
    //This function is to lock onto the AprilTag on the Hub
    System.out.println("[TURRET] Set to AprilTag on Hub");
  } 

public void startStopLauncherMotors(){
  //Starts or stops the launcher motors
  System.out.println("[TURRET] Launcher motors started/stopped");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
