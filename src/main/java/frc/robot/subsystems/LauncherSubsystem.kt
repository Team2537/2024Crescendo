import com.revrobotics.*
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.Constants
import frc.robot.Robot
import lib.math.units.RotationVelocity
import lib.math.units.into
import kotlin.math.min

object LauncherSubsystem : SubsystemBase() {

    /** Motor for the top flywheels */
    val topFlywheels: CANSparkFlex = CANSparkFlex(Constants.LauncherConstants.TOP_FLYWHEELS,
        CANSparkLowLevel.MotorType.kBrushless)

    /** Motor for the bottom flywheels */
    val bottomFlywheels: CANSparkFlex = CANSparkFlex(Constants.LauncherConstants.BOTTOM_FLYWHEELS,
        CANSparkLowLevel.MotorType.kBrushless)

    /** Motor for the roller that feeds the launcher */
    val rollerMotor: CANSparkMax = CANSparkMax(Constants.LauncherConstants.ROLLER_MOTOR,
        CANSparkLowLevel.MotorType.kBrushless)

//    val leftNoteDetector: DigitalInput = DigitalInput(Constants.LauncherConstants.LEFT_NOTE_DETECTOR)
    /** Sensor placed on the right side of the launcher to detect the note */
    val rightNoteDetector: DigitalInput = DigitalInput(Constants.LauncherConstants.RIGHT_NOTE_DETECTOR)

    /** The current stored setpoint for the roller */
    var setPoint: Double = 0.0

    /** Shuffleboard tab for the Launcher subsystem */
    val tab = Shuffleboard.getTab("Launcher")

    /** PID Controller for the flywheel */
    val flywheelPIDController: SparkPIDController = topFlywheels.pidController
    /** Feedforward controller for the flywheel */
    val flywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(
        0.0, 0.0, 0.0
    ) // TODO: Get real values


    /** Trigger for when the note is detected */
    val noteTrigger: Trigger

    /**
     * The system identification routine for the launcher subsystem.
     */
    val routine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage> ->
//                topFlywheels.setVoltage(volts.into(Units.Volts))
                bottomFlywheels.setVoltage(volts.into(Units.Volts))
            },
            { log: SysIdRoutineLog ->
//                log.motor("topFlywheels")
//                    .voltage(Units.Volts.of(topFlywheels.appliedOutput * topFlywheels.busVoltage))
//                    .angularPosition(Units.Rotations.of(topFlywheels.encoder.position))
//                    .angularVelocity(Units.RPM.of(topFlywheels.encoder.velocity))
//                    .current(Units.Amps.of(topFlywheels.outputCurrent))
                log.motor("bottomFlywheels")
                    .voltage(Units.Volts.of(bottomFlywheels.appliedOutput * bottomFlywheels.busVoltage))
                    .angularPosition(Units.Rotations.of(bottomFlywheels.encoder.position))
                    .angularVelocity(Units.RPM.of(bottomFlywheels.encoder.velocity))
                    .current(Units.Amps.of(bottomFlywheels.outputCurrent))
            },
            this
        )
    )

    /** Feedforward controller for the top flywheel */
    val topFlywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(-0.20832, 0.11109, 0.024896)
    /** Feedforward controller for the bottom flywheel */
    val bottomFlywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.035079, 0.10631, 0.0080339)

    init {
        // Reset the motors to factory defaults to ensure that they are in a known state
        topFlywheels.restoreFactoryDefaults()
        bottomFlywheels.restoreFactoryDefaults()
        rollerMotor.restoreFactoryDefaults()

        // Invert the bottom flywheels so that both sets of flywheels are pushing or pulling together
        bottomFlywheels.inverted = true

        // Set the current limits so that the motors don't burn out
        topFlywheels.setSmartCurrentLimit(40)
        bottomFlywheels.setSmartCurrentLimit(40)
        rollerMotor.setSmartCurrentLimit(40)

        // Set the conversion factors for the encoders so that they are in rotations, with no gearing
        topFlywheels.encoder.positionConversionFactor = 1.0
        bottomFlywheels.encoder.positionConversionFactor = 1.0
        topFlywheels.encoder.velocityConversionFactor = 1.0
        bottomFlywheels.encoder.velocityConversionFactor = 1.0

        // Set the motors to brake mode so that they don't move (as easily) when disabled
        topFlywheels.setIdleMode(CANSparkBase.IdleMode.kBrake)
        bottomFlywheels.setIdleMode(CANSparkBase.IdleMode.kBrake)

        // Set the PID values for the roller
        rollerMotor.pidController.p = 0.25
        rollerMotor.pidController.i = 0.0
        rollerMotor.pidController.d = 0.0

        // Create a trigger for when the note is detected, and debounce it so that it doesn't trigger multiple times
        noteTrigger = Trigger() {
            !rightNoteDetector.get() && Robot.isEnabled
        }.debounce(0.1)

        // So much logging oh my god I forgot there was so much logging
        Shuffleboard.getTab("Launcher").addBoolean("Note Detected") { noteTrigger.asBoolean }
        Shuffleboard.getTab("Launcher").addDouble("Position") { getRollerPosition() }
        Shuffleboard.getTab("Launcher").addDouble("Setpoint") { setPoint }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Top Flywheel Position") { topFlywheels.encoder.position }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Top Flywheel Velocity") { topFlywheels.encoder.velocity }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Bottom Flywheel Position") { bottomFlywheels.encoder.position }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Bottom Flywheel Velocity") { bottomFlywheels.encoder.velocity }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Top Flywheel Voltage") {topFlywheels.appliedOutput * topFlywheels.busVoltage }
        Shuffleboard.getTab("Launcher Sysid").addDouble("Bottom Flywheel Voltage") {bottomFlywheels.appliedOutput * bottomFlywheels.busVoltage }

        // Burn the settings to the Spark Maxes so that they persist across power cycles
        topFlywheels.burnFlash()
        bottomFlywheels.burnFlash()
        rollerMotor.burnFlash()

        // Set the encoder positions to 0 so that they are in a known state
        rollerMotor.encoder.setPosition(0.0)
        topFlywheels.encoder.setPosition(0.0)
        bottomFlywheels.encoder.setPosition(0.0)
    }

    /**
     * Set the brake mode for the flywheels
     * @param brake Whether to set the flywheels to brake mode or coast mode
     */
    fun setFlywheelBrake(brake: Boolean){
        if(brake){
            topFlywheels.setIdleMode(CANSparkBase.IdleMode.kBrake)
            bottomFlywheels.setIdleMode(CANSparkBase.IdleMode.kBrake)
        } else {
            topFlywheels.setIdleMode(CANSparkBase.IdleMode.kCoast)
            bottomFlywheels.setIdleMode(CANSparkBase.IdleMode.kCoast)
        }
    }

    /**
     * Set the flywheel speeds to a raw duty cycle value
     * @param rawSpeed The raw duty cycle value to set the flywheels to
     */
    fun setFlywheelSpeeds(rawSpeed: Double) {
        topFlywheels.set(rawSpeed)
        bottomFlywheels.set(rawSpeed)
    }

    /**
     * Set the roller speed to a raw duty cycle value
     * @param rawSpeed The raw duty cycle value to set the roller to
     */
    fun setRollerSpeed(rawSpeed: Double) {
        rollerMotor.set(rawSpeed)
    }

    /**
     * Set the flywheel speeds to a velocity
     * @param velocity The velocity to set the flywheels to
     */
    fun setFlywheelVelocity(velocity: RotationVelocity){
        // Get the lowest velocity of the two flywheels
        val currentV = min(topFlywheels.encoder.velocity, bottomFlywheels.encoder.velocity)
        // Calculate the error between the current velocity and the setpoint
        val error = velocity.into(Units.RPM) - currentV
        // Calculate the extra voltage to apply to the flywheels for error correction
        val extraVoltage = error * 0.002

        // Use the feedforward controller to calculate the voltage to apply to the flywheels.
        // This is the feedforward voltage plus the extra voltage for error correction
        topFlywheels.setVoltage(topFlywheelFeedforward.calculate(velocity into Units.RotationsPerSecond) + extraVoltage);
        bottomFlywheels.setVoltage(bottomFlywheelFeedforward.calculate(velocity into Units.RotationsPerSecond) + extraVoltage)
    }

    /**
     * Set the flywheel speeds to a voltage
     * @param voltage The voltage to set the flywheels to
     */
    fun setFlywheelVoltage(voltage: Double){
        topFlywheels.setVoltage(voltage)
    }

    /**
     * Set the roller speed to a velocity
     * @param velocity The velocity to set the roller to
     */
    fun stopFlywheels() {
        topFlywheels.set(0.0)
        bottomFlywheels.set(0.0)
    }

    /**
     * Set the roller speed to a velocity
     * @param velocity The velocity to set the roller to
     */
    fun stopRoller() {
        rollerMotor.set(0.0)
    }

    /**
     * Set the roller target position to a setpoint
     * @param position The position to set the target to
     */
    fun setRollerPosition(position: Double) {
        // Send a request to the Spark Max to set the position of the roller
        rollerMotor.pidController.setReference(position, CANSparkBase.ControlType.kPosition)
        // Store the setpoint so that it can be accessed later for logging
        setPoint = position
    }

    /**
     * Get the current position of the roller
     * @return The current position of the roller
     */
    fun getRollerPosition(): Double {
        return rollerMotor.encoder.position
    }

    /**
     * Returns whether the note is being detected
     * @return Whether the note is being detected
     */
    fun noteDetected(): Boolean {
        return !rightNoteDetector.get()
    }

    /**
     * Creates a command to run a dynamic system identification routine
     * @param direction The direction to run the routine in
     * @return The command to run the routine
     */
    fun dynamicSysIDRoutine(direction: Direction): Command? {
        return routine.dynamic(direction)
    }

    /**
     * Creates a command to run a quasi-static system identification routine
     * @param direction The direction to run the routine in
     * @return The command to run the routine
     */
    fun quasiStaticSysIDRoutine(direction: Direction): Command? {
        return routine.quasistatic(direction)
    }
}