package frc.robot.subsystems;
//Cheif Delphi "Elevator subsystem example code"
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// TODO: Add placeholder methods to raise the Elevator to L2 and L3 and L4


// TODO: Add a method to return the Elevator to L1

public class ElevatorSubsystem extends SubsystemBase{
    private final SparkFlex primaryMotor;
    private final SparkFlex followerMotor;
    private final RelativeEncoder encoder;
    //TODO: is this the right kind of sensor for the bottom limit?
    private final DigitalInput bottomLimit;
    private final PIDController pidController;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;
    private final TrapezoidProfile profile;
    /**
     * determines if we are in autoscore or manual score mod
     */
    private boolean isInManualMode;

    /**
     * determines if the elevator is enabled
     */
    private boolean isEnabled;

    private ElevatorPosition currentTarget = ElevatorPosition.DOWN;
    private boolean isHomed = false;
    private double setpoint = 0.0;
    SparkMaxConfig resetConfig = new SparkMaxConfig();

    /**
     * Current position of the elevator in inches
     */
    double currentPos;

    public enum ElevatorPosition {
        DOWN(ElevatorConstants.kDownPos),
        POSITION_1(ElevatorConstants.kL1),
        POSITION_2(ElevatorConstants.kL2),
        POSITION_3(ElevatorConstants.kL3),
        POSITION_4(ElevatorConstants.kL4);

        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }

    public ElevatorSubsystem() {
        isEnabled = true;
        isInManualMode = false;
        Shuffleboard.getTab("Elevator").add("Elevator Position",currentPos);

        primaryMotor = new SparkFlex(ElevatorConstants.KLeftElevatorID, MotorType.kBrushless);
        followerMotor = new SparkFlex(ElevatorConstants.KRightElevatorID, MotorType.kBrushless);
        
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(primaryMotor, false);

        // Configure follower
        followerMotor.configure(followerConfig, null, null); 
        
        encoder = primaryMotor.getEncoder();
        bottomLimit = new DigitalInput(ElevatorConstants.kLimitSwitchPort);

        resetConfig.idleMode(IdleMode.kBrake);
        //TODO: Ask the Great Bearded One if these should be set here, or are they just set on the physical motor controller?


        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);

        constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.kMaxVelocity,
            ElevatorConstants.kMaxAcceleration
        );
        
        pidController = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD
        );
        
        pidController.setTolerance(0.5); // 0.5 inches position tolerance
        
        // Initialize states and profile
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);
        
        configureMotors();
    }

    private void configureMotors() {
        // Primary motor configuration
        primaryMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
        
        // Follower motor configuration
        followerMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
    }

    @Override
        public void periodic() {
        currentPos = encoder.getPosition() / ElevatorConstants.kCountsPerInch;
        currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop
        //System.out.println("The elevator is at " + getHeightInches());
        //System.out.println("limit switch engaged " + !bottomLimit.get());
        if (!bottomLimit.get()) {
            handleBottomLimit();
        }

        if (getHeightInches() > ElevatorConstants.kMaxPos) {
            stopMotors();
        }

        // Only run control if homed
        if (isHomed) {
            double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
            double ff = calculateFeedForward(currentState);
            
            double outputPower = MathUtil.clamp(
                pidOutput + ff,
                -ElevatorConstants.kMax_output,
                ElevatorConstants.kMax_output
            );
            
            primaryMotor.set(outputPower);
        }

        // Update SmartDashboard
        updateTelemetry();
    }

    private void handleBottomLimit() {
        //System.out.println("Elevator is homed");
        stopMotors();
        encoder.setPosition(ElevatorConstants.kBottomPos * ElevatorConstants.kCountsPerInch);
        isHomed = true;
        setpoint = ElevatorConstants.kBottomPos;
        currentState = new TrapezoidProfile.State(ElevatorConstants.kBottomPos, 0);
        goalState = new TrapezoidProfile.State(ElevatorConstants.kBottomPos, 0);
        pidController.reset();
    }

    public void stopMotors() {
        primaryMotor.set(0);
        pidController.reset();
    }

    public boolean isAtHeight(double targetHeightInches) {
        // Check if the elevator is within a small tolerance of the target height
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - targetHeightInches) < ElevatorConstants.kPosTolerance;
    }
    

    private double calculateFeedForward(TrapezoidProfile.State state) {
        // kS (static friction), kG (gravity), kV (velocity),
        return ElevatorConstants.kS * Math.signum(state.velocity) +
               ElevatorConstants.kG +
               ElevatorConstants.kV * state.velocity;
    }

    public void setPositionInches(double inches) {
        if (!isHomed && inches > 0) {
            System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
            return;
        }

        setpoint = MathUtil.clamp(
            inches,
            ElevatorConstants.kMinPos,
            ElevatorConstants.kMaxPos
        );
        
        // Update goal state for motion profile
        goalState = new TrapezoidProfile.State(setpoint, 0);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Elevator Target", setpoint);
        SmartDashboard.putBoolean("Elevator Homed", isHomed);
        SmartDashboard.putString("Elevator State", currentTarget.toString());
        SmartDashboard.putNumber("Elevator Current", primaryMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Velocity", currentState.velocity);
    }

    public double getHeightInches() {
        return Math.abs(encoder.getPosition() / ElevatorConstants.kCountsPerInch);
    }

    public void homeElevator() {
        primaryMotor.set(-0.1); // Slow downward movement until bottom limit is hit
        if (!bottomLimit.get()) {
            handleBottomLimit();
        }
    }

    public boolean isAtPosition(ElevatorPosition position) {
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - position.positionInches) < 0.5;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public ElevatorPosition getCurrentTarget() {
        return currentTarget;
    }

    public void setManualPower(double power) {
        System.out.println("set manual power");
        // Disable PID control when in manual mode
        pidController.reset();
        currentState = new TrapezoidProfile.State(getHeightInches(), 0);
        goalState = new TrapezoidProfile.State(getHeightInches(), 0);
        
        if (!isHomed && power < 0) {
            power = 0;
        }
        
        if (getHeightInches() >= ElevatorConstants.kMaxPos && power > 0) {
            power = 0;
        }
        //TODO: turned this off. probably a bad idea
        // if (!bottomLimit.get() && power < 0) {
        //     power = 0;
        // }
        
        primaryMotor.set(MathUtil.clamp(power, -ElevatorConstants.kMax_output, ElevatorConstants.kMax_output));
    }



    public void toggleManualMode() {
        isInManualMode = !isInManualMode;
        System.out.println("Is in manual"+isInManualMode);
    }

    public boolean getManualMode() {
            return isInManualMode;
    }

    public void setManualMode(boolean mode){
            isInManualMode = mode;
    }

     // Getter for isEnabled
     public boolean isEnabled() {
        return isEnabled;
    }

    // Setter for isEnabled
    public void setEnabled(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }
    public void moveElevator(double speed) {
        // Ensure the speed is within the valid range
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        primaryMotor.set(speed);
        followerMotor.set(speed);
        System.out.println("Elevator moving at speed: " + speed);
    }
}



