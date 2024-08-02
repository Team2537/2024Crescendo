package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Dimensionless
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.PivotConstants
import lib.math.units.*
import lib.putMap

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
        tab.addBoolean("Homing Sensor") { this.isUpright }

        // Reset the relative encoder to 0 so it's in a known state
        zeroEncoder()

        // Set current limits for the motor
        pivotMotor.setSmartCurrentLimit(40)

        // Add map of distances to angles for auto-aiming
        // This isn't really used in the current code, but it's left in for backwards compatibility
        // Or something ¯\_(ツ)_/¯
        autoAimTree.putMap(PivotConstants.distanceMap)
    }

    /**
     * Get the interpolated angle for a given position
     * @param position The distance to get the angle for
     * @return The angle for the given distance
     */
    fun getInterpolatedAngle(position: Span): Rotation {
        // FIXME WHAT UNITS ARE THEYYYY???
        return autoAimTree[position into Meters].radians // No one bothered to say what unit the map was in??????
    }

    /**
     * Gets the position of the pivot. By default, this will be the relative position.
     *
     * Complain to Falon or something if you want to change that
     *
     * @see relativePosition
     * @see absolutePosition
     */
    val position: Rotation
        get() = relativePosition

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
            // The conversion ratio supposedly goes to degrees.
            // Blame falon if it's wrong
            return relativeEncoder.position.degrees
        }

    /**
     * Sync the relative encoder to the absolute encoder
     */
    fun syncRelative(){
        // They should be in the same unit, so avoid garbage measures
        relativeEncoder.position = absoluteEncoder.distance
    }

    // According to the docs for the homingSensor object, it is supposed to trigger when the
    // pivot is upright, so, once again, complain to Falon if the previous comments are wrong
    // Previously was getHomingSensor()
    /**
     * Get the homing sensor value
     * @return The value of the homing sensor
     */
    val isUpright get() = homingSensor.get()

    /**
     * Zero the relative encoder
     */
    fun zeroEncoder() {
        relativeEncoder.position = 0.0
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
        pivotMotor.set(speed into edu.wpi.first.units.Units.Percent)
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
    fun holdArm(position: Rotation){
        pivotPID.setReference(
            position into Degrees,
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.calculate(position into Radians, 0.0)
        )
    }

    /** Stop the pivot */
    fun stop() {
        pivotMotor.stopMotor()
    }

}