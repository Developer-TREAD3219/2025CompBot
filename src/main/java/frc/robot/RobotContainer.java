// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TODO: Finish the robot and party
// TODO: Make a Predive Checklist. What do we need to do to home the bot, making sure we chose the correct auto etc. 

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.coralDeliveryConstants;
//TODO: Reminder to import any new subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CoralDeliverySubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.BeginEndMatch;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // : Add the subsystems to the RobotContainer
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  //TODO: need sensor at bottom to reset controller value

  public class ElevatorSensor {
    
  }

  private final CoralDeliverySubsystem m_CoralDeliverySubsystem = new CoralDeliverySubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final LimeLightSubsystem m_LimeLightSubsystem = new LimeLightSubsystem(m_robotDrive);
  private final Servo m_intakeServo = new Servo(coralDeliveryConstants.kIntakeServoID);


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(OIConstants.kGunnerControllerPort);

  private final SendableChooser<Command> autoChooser;




  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Supresses the "No Joystick Connected" Spam

    if (RobotBase.isSimulation() || DriverStation.isTest()) {
  DriverStation.silenceJoystickConnectionWarning(true);
}
    // Configure the button bindings
    configureButtonBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    
    //TODO: Add our commands here
    //TODO: Intake Coral: 


            //This should require just the Coral intake and some generic sensor to detect a stop
            //we want to be able to intake and stop when we have a coral
      //boolean that checks if we have a coral
    //TODO: We need to plan out how were going to structure our coral scoring commands
          //this is going to require the Elavator CoralDelivery Drive and Limelight systems
          // we want to be able to score a left and right version of L2 L3 and L4 as well as L1 which doesn't require a side(we may still want one or we may want to handle scoring L1 in an entirely different way discussion topic)
          // This should probably all be in 1 command with logic to choose between the 7 different scoring configurations
          // Keeping it all in 1 command means we only have to adjust one command as we fine tune
          // We are going to want a way to abort the command incase there is a technical difficulty
          // It would be really cool if we made the controllers rumble when this action is done so the driver knows they can go
          // Make sure it lowers back down so we can drive fast without tipping
    
    //TODO: We need a command that switches us into an end game mode
          //This is going to use the intake climber elevator maybe(to bring it home) and a Camera we haven't set up yet.
          //We want to open the intake enable the climber and switch from the Limelight to our climbcam
          //WE DO NOT WANT TO ACCIDENTLY TRIGGER THIS once we open that door its not closing and we can't score anymore


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() { 
        // Drive Controller inputs
        // TODO: Add button mappings for the driver controller
        // The RB button on the driver controller locks our wheels in the X position if we held 
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  
// LT + RT + Button:A= Open Trap Door during Climb
        
    // Y button makes whatever direction the robot is facing the new forward
    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new RunCommand(() -> m_robotDrive.resetYaw(), m_robotDrive));

     // Define the Trigger
     Trigger EndTriggerStart = new Trigger(this::EndGameStartRequested);

     // Bind the Trigger to the End Game Start
     EndTriggerStart.onTrue(new BeginEndMatch(m_ElevatorSubsystem, m_ClimberSubsystem, m_intakeServo));

// Many of these are going to need their own commmands


//Gunner Control

// LJoystick/ while in manual= Move Elevator Manually, In case of Auto Breaking
//LT= Score Left Coral 
//RT= Score Right Coral
//X= Level 1 for Coral Auto (should automatically got to selected Level)
//Y= Level 2 for Coral Auto (should automatically got to selected Level)
//B= Level 3 for Coral Auto (should automatically got to selected Level)
//A= Level 4 for Coral Auto (should automatically got to selected Level)
//DPad Up= Going Up to Selected Level and should be Combined with Level Auto
//DPad Down= Going Down to Selected Level and should be Combined with Level Auto
//Start= Toggle between Manual and Automatic mode.

    // TODO: Add button mappings for the gunner controller

    new JoystickButton(m_gunnerController, XboxController.Button.kStart.value)
    .onTrue(new InstantCommand(() -> m_ElevatorSubsystem.toggleManualMode()));


    // Using Left Joystick while in Manual mode in order to move the elevator manually.
m_ElevatorSubsystem.setDefaultCommand(new RunCommand(() -> {
  if (m_ElevatorSubsystem.getManualMode()) {
      double speed = -m_gunnerController.getLeftY(); // Invert Y-axis if necessary
      m_ElevatorSubsystem.moveElevator(speed);
  }
}, m_ElevatorSubsystem));

// Define the Trigger
// Bind the Trigger to the AutoScoreCommand
Trigger autoScoreTrigger = new Trigger(this::autoScoreCommandRequested);
autoScoreTrigger.onTrue(new AutoScoreCommand(m_ElevatorSubsystem, m_gunnerController));
//LT= Score Left Coral

// gunner dpad right triggers manual intake slow
Trigger intakeTrigger = new Trigger(this::intakeRequested);
intakeTrigger.whileTrue(new RunCommand(() -> m_CoralDeliverySubsystem.manualSpin(0.5), m_CoralDeliverySubsystem));
intakeTrigger.onFalse(new RunCommand(() -> m_CoralDeliverySubsystem.stopMotor(), m_CoralDeliverySubsystem));


Trigger outtakeTrigger = new Trigger(this::outtakeRequested);
outtakeTrigger.whileTrue(new RunCommand(() -> m_CoralDeliverySubsystem.manualSpin(1), m_CoralDeliverySubsystem));
outtakeTrigger.onFalse(new RunCommand(() -> m_CoralDeliverySubsystem.stopMotor(), m_CoralDeliverySubsystem));
}
  // Method to get the time remaining in the match
  public double getMatchTime() {
    return DriverStation.getMatchTime();
}

// Check if we have a valid button combo for auto score
private boolean autoScoreCommandRequested() {
    return (m_gunnerController.getAButton() ||
            m_gunnerController.getYButton() ||
            m_gunnerController.getXButton() ||
            m_gunnerController.getBButton() ) &&
           (m_gunnerController.getLeftTriggerAxis() > 0.9 ||
            m_gunnerController.getRightTriggerAxis() > 0.9);
}
// check if we are trying to start the end game
public boolean EndGameStartRequested() {
  return (m_driverController.getLeftTriggerAxis() > 0.9 &&
          m_driverController.getRightTriggerAxis() > 0.9 &&
          m_driverController.getAButton()) &&
          (
          getMatchTime() < 30 || 
          RobotBase.isSimulation() || 
          DriverStation.isTest());
  }

  //Check if dpad right is pressed on the gunner controller
  public boolean intakeRequested(){
    return m_gunnerController.getPOV() == 90;
  }

  public boolean outtakeRequested(){
    return m_gunnerController.getPOV() == 270;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
