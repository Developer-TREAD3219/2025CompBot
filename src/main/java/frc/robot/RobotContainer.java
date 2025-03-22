// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// TODO: Finish the robot and party
// TODO: Make a Predive Checklist. What do we need to do to home the bot, making sure we chose the correct auto etc. 

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
//import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.coralDeliveryConstants;
import frc.robot.commands.BeginEndMatch;
import frc.robot.commands.goToElevatorL2;
import frc.robot.commands.ReefAlignment;
import frc.robot.commands.CoralDelivery.CoralIntakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralDeliverySubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public Pigeon2 m_Pigeon;
  public DriveSubsystem m_robotDrive;
  public CoralDeliverySubsystem m_CoralDeliverySubsystem;
  public ClimberSubsystem m_ClimberSubsystem;
  public ElevatorSubsystem  m_ElevatorSubsystem;
  public LimeLightSubsystem m_LimeLightSubsystem;
  //public Servo m_intakeServo;
  public Talon  m_hatchMotor;
  public CANBus m_CanBus;


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_gunnerController = new XboxController(OIConstants.kGunnerControllerPort);

  private final SendableChooser<Command> autoChooser;




  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
      // The robot's subsystems
  // : Add the subsystems to the RobotContainer
  m_CanBus = new CANBus();
  m_Pigeon = new Pigeon2(15, m_CanBus);
  m_robotDrive = new DriveSubsystem(m_Pigeon);
  m_ClimberSubsystem = new ClimberSubsystem();
  m_ElevatorSubsystem = new ElevatorSubsystem();
  m_LimeLightSubsystem = new LimeLightSubsystem(m_robotDrive);
  // m_intakeServo = new Servo(coralDeliveryConstants.kIntakeServoID);
  m_hatchMotor = new Talon(coralDeliveryConstants.kIntakeServoID);
  m_CoralDeliverySubsystem = new CoralDeliverySubsystem(m_ElevatorSubsystem);
    

    // Supresses the "No Joystick Connected" Spam

  //  if (RobotBase.isSimulation() || DriverStation.isTest()) {
  DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(Math.pow(m_driverController.getLeftY(), 5), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.pow(m_driverController.getLeftX(), 5), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
              true),
            m_robotDrive));

        // Configure the button bindings
        configureButtonBindings();
        addShuffleboardWidgets();

      // Auto Named Commands for path planner
      NamedCommands.registerCommand("RaiseToL2", Commands.print("we did it"));
  }

  /**
   * Button Bindings
   */
  private void configureButtonBindings() {
  

    //   _____       _                   _____            _             _     
    //  |  __ \     |_|                 / ____|          | |           | |    
    //  | |  | |_ __ ___   _____ _ __  | |     ___  _ __ | |_ _ __ ___ | |___ 
    //  | |  | | '__| \ \ / / _ \ '__| | |    / _ \| '_ \| __| '__/ _ \| / __|
    //  | |__| | |  | |\ V /  __/ |    | |___| (_) | | | | |_| | | (_) | \__ \
    //  |_____/|_|  |_| \_/ \___|_|     \_____\___/|_| |_|\__|_|  \___/|_|___/
 
      //Drive controller left bumper rotates the climber
      new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      .whileTrue(new RunCommand(() -> m_ClimberSubsystem.rotateClimber(1), m_ClimberSubsystem))
      .onFalse(new InstantCommand(() -> m_ClimberSubsystem.stopClimber(), m_ClimberSubsystem));

      // The RB button rotates the climber in the other direction
      new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
      .whileTrue(new RunCommand(() -> m_ClimberSubsystem.rotateClimber(-1), m_ClimberSubsystem))
      .onFalse(new InstantCommand(() -> m_ClimberSubsystem.stopClimber(), m_ClimberSubsystem));

      // The X button attempts to alling on the right reef
      new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whileTrue(new ReefAlignment(m_LimeLightSubsystem, m_robotDrive, m_ElevatorSubsystem, true));
      
      // The B button attempts to alling on the right reef
      new JoystickButton(m_driverController, XboxController.Button.kB.value)
      .whileTrue(new ReefAlignment(m_LimeLightSubsystem, m_robotDrive, m_ElevatorSubsystem, false));

      // LT + RT + Button:A= Open Trap Door during Climb
      Trigger endTriggerStart = new Trigger(this::EndGameStartRequested);
      // Bind the Trigger to the End Game Start
      endTriggerStart.onTrue(new BeginEndMatch(m_ElevatorSubsystem, m_ClimberSubsystem, m_hatchMotor)
      .withTimeout(2));  // was m_intakeServo




      


  //    _____                                _____            _             _     
  //   / ____|                              / ____|          | |           | |    
  //  | |  __ _   _ _ __  _ __   ___ _ __  | |     ___  _ __ | |_ _ __ ___ | |___ 
  //  | | |_ | | | | '_ \| '_ \ / _ \ '__| | |    / _ \| '_ \| __| '__/ _ \| / __|
  //  | |__| | |_| | | | | | | |  __/ |    | |___| |_| | | | | |_| | | (_) | \__ \
  //   \_____|\__,_|_| |_|_| |_|\___|_|     \_____\___/|_| |_|\__|_|  \___/|_|___/

      // gunner dpad up triggers auto intake
      Trigger autoIntakeTrigger = new Trigger(this::autoIntakeRequested);
      autoIntakeTrigger.onTrue(new CoralIntakeCommand(m_CoralDeliverySubsystem, m_driverController));

      //gunner dpad left manual spins at outake speed
      Trigger outtakeTrigger = new Trigger(this::outtakeRequested);
      outtakeTrigger.whileTrue(new RunCommand(() -> m_CoralDeliverySubsystem.manualSpin(coralDeliveryConstants.kOuttakeSpeed), m_CoralDeliverySubsystem));
      outtakeTrigger.onFalse(new RunCommand(() -> m_CoralDeliverySubsystem.stopMotor(), m_CoralDeliverySubsystem));
      outtakeTrigger.onFalse(new RunCommand(() -> m_ElevatorSubsystem.goToElevatorStow(), m_ElevatorSubsystem));
  
    /*
    * ELEVATOR COMMANDS
    */

    //A sets to L1/home
    new JoystickButton(m_gunnerController, XboxController.Button.kA.value)
    .onTrue(new RunCommand(() -> m_ElevatorSubsystem.goToElevatorStow(), m_ElevatorSubsystem));

    //X sets to L2
    new JoystickButton(m_gunnerController, XboxController.Button.kX.value)
    .onTrue(new InstantCommand(() -> m_ElevatorSubsystem.goToElevatorL2(), m_ElevatorSubsystem));

    //Y sets to L3
    new JoystickButton(m_gunnerController, XboxController.Button.kY.value)
    .onTrue(new RunCommand(() -> m_ElevatorSubsystem.goToElevatorL3(), m_ElevatorSubsystem));

    //B sets to L4
    new JoystickButton(m_gunnerController, XboxController.Button.kB.value)
    .onTrue(new RunCommand(() -> m_ElevatorSubsystem.goToElevatorL4(), m_ElevatorSubsystem));

    }
  // Method to get the time remaining in the match
  public double getMatchTime() {
    return DriverStation.getMatchTime();
}
/*
 * Set up Shuffleboard controls
 */
private void  addShuffleboardWidgets(){
  Shuffleboard.getTab("Elevator")
  .add("Home Elevator", new InstantCommand(m_ElevatorSubsystem::goToElevatorStow));
}
// check if we are trying to start the end game
public boolean EndGameStartRequested() {
  return (m_driverController.getLeftTriggerAxis() > 0.9 &&
          m_driverController.getRightTriggerAxis() > 0.9 &&
          m_driverController.getAButton())
          //  &&
          // (
          // DriverStation.getMatchTime() < 45 || 
          // RobotBase.isSimulation() || 
          // DriverStation.isTest()
          // )
          ;
  }

  //Check if dpad right is pressed on the gunner controller
  public boolean outtakeRequested(){
    return m_gunnerController.getPOV() == 270;
  }
  public Boolean autoIntakeRequested(){
    return m_gunnerController.getPOV() == 0;
  }

//                 _                                              
//      /\        | |                                             
//     /  \  _   _| |_ ___  _ __   ___  _ __ ___   ___  _   _ ___ 
//    / /\ \| | | | __/ _ \| '_ \ / _ \| '_ ` _ \ / _ \| | | / __|
//   / ____ \ |_| | || (_) | | | | (_) | | | | | | (_) | |_| \__ \
//  /_/    \_\__,_|\__\___/|_| |_|\___/|_| |_| |_|\___/ \__,_|___/
//
  public Command getAutonomousCommand() {
   //return autoChooser.getSelected();
    Command m_autonomousCommand;
    
    m_autonomousCommand = new PathPlannerAuto(autoChooser.getSelected())     
    .andThen(() -> m_robotDrive.drive(0.1, 0, 0, false))
    .andThen(Commands.waitSeconds(0.3))
    .andThen(() -> m_robotDrive.drive(0,0,0, false))
    .andThen(() -> m_ElevatorSubsystem.goToElevatorL4(), m_ElevatorSubsystem)
    .andThen(Commands.waitSeconds(2))
    .andThen(() -> m_robotDrive.drive(-0.1, 0 , 0, false))
    .andThen(Commands.waitSeconds(0.3))
    .andThen(() -> m_robotDrive.drive(0,0,0, false))
    .andThen(() -> m_CoralDeliverySubsystem.spinMotor(coralDeliveryConstants.kOuttakeSpeed), m_CoralDeliverySubsystem)
    .andThen(Commands.waitSeconds(2))
    .andThen(() -> m_CoralDeliverySubsystem.stopMotor(), m_CoralDeliverySubsystem);
    // .andThen(() -> m_ElevatorSubsystem.goToElevatorStow(), m_ElevatorSubsystem);
    
    return m_autonomousCommand;
    };
  }
  

