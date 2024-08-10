package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.PivotConstants
import lib.math.units.*
import lib.putMap
import java.util.function.Supplier

object PivotSubsystem : SubsystemBase() {

    /** The motor for the pivot */
    private val pivotMotor: CANSparkMax = CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT,
        CANSparkLowLevel.MotorType.kBrushless)

    /** The absolute encoder for the pivot */
    private val absoluteEncoder: DutyCycleEncoder = DutyCycleEncoder(PivotConstants.ABSOLUTE_ENCODER_PORT)

    /** The relative encoder for the pivot, contained within the SparkMax */
    private val relativeEncoder: RelativeEncoder = pivotMotor.encoder

    /** The PID controller for the pivot */
    private val pivotPID: SparkPIDController = pivotMotor.pidController

    /** The homing sensor for the pivot, to trigger when the pivot is upright */
    private val homingSensor: DigitalInput = DigitalInput(PivotConstants.HOMING_SENSOR_PORT)

    /** The Shuffleboard tab for the pivot subsystem */
    private val tab: ShuffleboardTab = Shuffleboard.getTab("Pivot")


    // NOTE: Everything relating to profiles and such are in the same unit as [setpoint], i.e. degrees
    private val oldState: TrapezoidProfile.State = TrapezoidProfile.State()
    private val goalState: TrapezoidProfile.State = TrapezoidProfile.State()
    /** The current trapezoid profile */
    private var profile: TrapezoidProfile = TrapezoidProfile(PivotConstants.armConstraints)
    private val profileTimer: Timer = Timer()

    private val deltaTimer: Timer = Timer()

    /**
     * The target position of the arm, in degrees. A value of [Double.NaN] indicates
     * that there is no target position.
     *
     * This variable is used strictly internally, so it may remain a double
     * for speed and memory. If you have a problem with this, buy the team
     * a rio with more ram or something.
     */
    private var setpoint: Double = Double.NaN

    // Distances from what?
    /**
     * The tree map for the auto-aiming of the pivot
     * This is a map of distances to angles
     */
    private val autoAimTree: InterpolatingDoubleTreeMap = InterpolatingDoubleTreeMap()

    /** Feedforward controller for the pivot */
    private val feedforward: ArmFeedforward = ArmFeedforward(
        PivotConstants.kS,
        PivotConstants.kG,
        PivotConstants.kV,
        PivotConstants.kA
    )

    init {
        // Register self for periodic() calls. This is recommended to happen in the
        // constructor of the subsystem
        CommandScheduler.getInstance().registerSubsystem(this)

        // Restore factory defaults for the motor
        pivotMotor.restoreFactoryDefaults()

        // Set the conversion factors for the encoders, so that they're in degrees after the gearbox and pulley ratio
        relativeEncoder.positionConversionFactor = PivotConstants.REL_ENCODER_CONVERSION
        absoluteEncoder.distancePerRotation = PivotConstants.ABS_ENCODER_CONVERSION
        absoluteEncoder.positionOffset = PivotConstants.ABSOLUTE_OFFSET

        // Set the PID constants for the pivot
        pivotPID.p = PivotConstants.kP
        pivotPID.i = PivotConstants.kI
        pivotPID.d = PivotConstants.kD
        pivotPID.ff = 0.0

        // Logging for Shuffleboard, position, velocity, voltage, and homing sensor
        tab.add("Throughbore Distance") { absolutePosition }
        tab.addDouble("Absolute Position") { absoluteEncoder.absolutePosition }
        tab.add("Relative Position") { relativePosition }
        tab.addDouble("Voltage Sent") { pivotMotor.appliedOutput * pivotMotor.busVoltage }
        tab.addBoolean("Homing Sensor") { this.isHome }

        // Reset the relative encoder to 0 so it's in a known state
        zeroEncoder()

        // Set current limits for the motor
        pivotMotor.setSmartCurrentLimit(40)

        // Add map of distances to angles for auto-aiming
        // This isn't really used in the current code, but it's left in for backwards compatibility
        // Or something ¯\_(ツ)_/¯
        autoAimTree.putMap(PivotConstants.distanceMap)

        setpoint = 0.0

        deltaTimer.start()

        profileTimer.start()
        updateProfile()
    }

    /**
     * Get the interpolated angle for a given position
     * @param position The distance to get the angle for
     * @return The angle for the given distance
     */
    fun getInterpolatedAngle(position: Span): Rotation {
        // You'll hate me for this, but its inches -> degrees
        // 0 degrees is fully upright, as it pivots back to shoot higher it goes towards 90
        return autoAimTree[position into Inches].degrees
    }

    /**
     * Checks if the pivot motor *should be* at rest
     */
    val isFinished: Boolean
        get() = setpoint.isNaN() || profile.isFinished(profileTime)

    /**
     * Gets the position of the pivot. By default, this will be the relative position.
     *
     * @see relativePosition
     * @see absolutePosition
     */
    var position: Rotation
        get() = relativePosition
        set(value) {
            targetPosition = value
        }

    /**
     * The position the pivot subsystem is trying to get to.
     */
    var targetPosition: Rotation
        get() {
            // I'm not sure how measures react to denormalized magnitudes, but I don't care to find out either
            return if(setpoint.isNaN() || setpoint.isInfinite()) position else setpoint.degrees
        }
        set(value) { setRotation(value) }

    /**
     * Get the absolute position of the pivot
     * @return The absolute position of the pivot
     */
    val absolutePosition: Rotation
        get() {
            return absoluteEncoder.distance.degrees
        }

    /**
     * Get the relative position of the pivot
     * @return The relative position of the pivot
     */
    val relativePosition: Rotation
        get() {
            // The conversion ratio goes to degrees.
            return relativeEncoder.position.degrees
        }

    /**
     * Sync the relative encoder to the absolute encoder
     */
    fun syncRelative(){
        // They should be in the same unit, so avoid garbage measures
        relativeEncoder.position = absoluteEncoder.distance
    }

    /**
     * Checks if the pivot is "home" by use of a sensor.
     *
     * @return The value of the homing sensor
     */
    val isHome get() = homingSensor.get()

    /**
     * Zero the relative encoder
     */
    fun zeroEncoder() {
        relativeEncoder.position = 0.0

        oldState.position = 0.0
        goalState.position = 0.0
    }

    // Both setRawSpeed are for percentage, and I feel that encouraging the creation of garbage measures
    // for something as simple as that is a bit stupid. For that reason, both a dimensionless option is
    // given, but the double option is just as valid
    /**
     * Set the speed of the pivot
     * @param speed The speed to set the pivot to [-1.0, 1.0]
     */
    fun setRawSpeed(speed: Double){
        pivotMotor.set(speed)
    }

    /**
     * Set the speed of the pivot
     * @param speed The speed to set the pivot to as a percentage
     */
    fun setRawSpeed(speed: Measure<Dimensionless>){
        pivotMotor.set(speed into Percent)
    }

    /**
     * Set a voltage amount to send to the pivot
     * @param voltage The voltage to send to the pivot
     */
    fun setVoltage(voltage: Measure<Voltage>) {
        pivotMotor.setVoltage(voltage into Volts)
    }

    /** Reset the absolute encoder */
    fun resetEncoder() {
        absoluteEncoder.reset()
    }

    /**
     * Set the position of the pivot using PID
     * @param position The position to set the pivot to
     */
    @Deprecated("Debugging, try to replace with `holdArm(Rotation)`", ReplaceWith("holdArm(position)"))
    fun setPIDPosition(position: Double) {
        pivotPID.setReference(position, CANSparkBase.ControlType.kPosition)
        println("Setting position to $position")
    }

    /**
     * Set the position of the pivot using PID with feedforward specialized to hold it in place
     * @param position The position to set the pivot to
     */
    fun setRotation(position: Rotation){
        val setpos = position into Degrees
        if(setpos != setpoint) {
            setpoint = setpos
            updateProfile()
        }
    }

    /**
     * Rotates the position of the pivot using PID with feedforward
     *
     * @param change The amount to rotate by.
     */
    fun rotate(change: Rotation){
       val rawChange = change into Degrees
       if(rawChange != 0.0){
           setpoint += rawChange
           updateProfile()
       }
    }

    /**
     * Updates the PID controller using trapezoid profiles and feedforward.
     */
    private fun update(){
        deltaTimer.reset()

        // If setpoint is NaN then things have been canceled
        if(setpoint.isNaN()){
            return
        }

        val targetState: TrapezoidProfile.State = getTargetState()
        pivotPID.setReference(
            targetState.position,
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.calculate(Math.toRadians(relativeEncoder.position + PivotConstants.HOME_DEGREES), targetState.velocity)
        )
    }

    /** The time, in seconds, that the current profile has been active for */
    private val profileTime get() = profileTimer.get()

    /** The time, in seconds, since the last call to [update] */
    private val deltaTime get() = deltaTimer.get()

    private fun getTargetState(): TrapezoidProfile.State {
        return if(profile.isFinished(profileTime))
            TrapezoidProfile.State(setpoint, 0.0)
        else
            profile.calculate(profileTime, oldState, goalState)
    }

    private fun updateProfile() {
        oldState.position = relativeEncoder.position
        oldState.velocity = relativeEncoder.velocity

        goalState.position = setpoint
        goalState.velocity = 0.0

        // I am irrationally upset at the fact that these profiles are not able to be
        // reset.
        profile = TrapezoidProfile(PivotConstants.armConstraints)

        profileTimer.reset()
    }

    /** Stop the pivot */
    fun stop() {
        pivotMotor.stopMotor()
        setpoint = Double.NaN
    }

    override fun periodic() {
        super.periodic()
        update() // always be updating
    }

    // Command Factories

    /**
     * Gets a [Command] that will tell the pivot to move to the position given by
     * [setter] each cycle.
     *
     * @param setter The specified retrieval of position. This will be run each cycle.
     *
     * @return A command that will set the pivot position.
     */
    fun getSetCommand(setter: Supplier<out Rotation>): Command {
        return Commands.run({
            setRotation(setter.get())
        }, this)
    }

    /**
     * Gets a [Command] that will tell the pivot to move to a predefined position.
     *
     * @param rotation The constant position to reset to
     */
    fun getSetCommand(rotation: Rotation): Command {
        return Commands.runOnce({
            setRotation(rotation)
        }, this)
    }

    /**
     * Gets a [Command] that will move the pivot by the change given by [changer]
     * each cycle. The change will be scaled to "per second" -- i.e., a constant
     * output of 45 degrees will move the pivot 45 degrees per second.
     *
     * @param changer The specified retrieval of change in position, scaled by delta
     * time. This will run each cycle.
     *
     * @return A command that will modify the pivot position.
     */
    fun getChangeCommand(changer: Supplier<out Rotation>): Command {
        return Commands.run({
            val change = changer.get()

            // Multiply by deltaTime to convert to a "per-second" basis
            val raw: Double = deltaTime * (change into Degrees)
            if (raw != 0.0) {
                setpoint += raw
                updateProfile()
            }
        }, this)
    }

    /**
     * Gets a [Command] that will move the pivot by the change given by [changer]
     * each cycle. The change will be scaled by the given ratio and then by delta
     * time. The default value of 1.0 will result in a "per second" ratio.
     *
     * @param changer The specified retrieval of change in position, scaled by delta
     * time. This will run each cycle.
     *
     * @return A command that will modify the pivot position.
     */
    fun getChangeCommand(ratio: Double, changer: Supplier<out Rotation>): Command {
        return Commands.run({
            val change = changer.get()

            // Multiply by deltaTime to convert to a "per-second" basis, then also a predefined ratio
            val raw: Double = ratio * deltaTime * (change into Degrees)
            if (raw != 0.0) {
                // Raw rotation because we have already converted to degrees
                setpoint += raw
                updateProfile()
            }
        }, this)
    }
}