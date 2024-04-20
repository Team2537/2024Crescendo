package frc.robot.subsystems

import com.revrobotics.*
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.PivotConstants
import lib.putMap

object PivotSubsystem : SubsystemBase() {

    /** The motor for the pivot */
    val pivotMotor: CANSparkMax = CANSparkMax(PivotConstants.PIVOT_MOTOR_PORT,
        CANSparkLowLevel.MotorType.kBrushless)

    /** The absolute encoder for the pivot */
    val absoluteEncoder: DutyCycleEncoder = DutyCycleEncoder(PivotConstants.ABSOLUTE_ENCODER_PORT)

    /** The relative encoder for the pivot, contained within the SparkMax */
    val relativeEncoder: RelativeEncoder = pivotMotor.encoder

    /** The PID controller for the pivot */
    val pivotPID: SparkPIDController = pivotMotor.pidController

    /** The homing sensor for the pivot, to trigger when the pivot is upright */
    val homingSensor: DigitalInput = DigitalInput(PivotConstants.HOMING_SENSOR_PORT)

    /** The Shuffleboard tab for the pivot subsystem */
    val tab: ShuffleboardTab = Shuffleboard.getTab("Pivot")

    /**
     * The tree map for the auto-aiming of the pivot
     * This is a map of distances to angles
     */
    val autoAimTree: InterpolatingDoubleTreeMap = InterpolatingDoubleTreeMap()

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
        tab.addDouble("Throughbore Distance") { getAbsolutePosition() }
        tab.addDouble("Absolute Position") { absoluteEncoder.absolutePosition }
        tab.addDouble("Relative Position") { getRelativePosition() }
        tab.addDouble("Voltage Sent") { pivotMotor.appliedOutput * pivotMotor.busVoltage }
        tab.addBoolean("Homing Sensor") { getHomingSensor() }

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
    fun getInterpolatedAngle(position: Double): Double {
        return autoAimTree.get(position)
    }

    /**
     * Get the absolute position of the pivot
     * @return The absolute position of the pivot
     */
    fun getAbsolutePosition(): Double {
        return absoluteEncoder.distance
    }

    /**
     * Get the relative position of the pivot
     * @return The relative position of the pivot
     */
    fun getRelativePosition(): Double {
        return relativeEncoder.position
    }

    /**
     * Sync the relative encoder to the absolute encoder
     */
    fun syncRelative(){
        relativeEncoder.setPosition(getAbsolutePosition())
    }

    /**
     * Get the homing sensor value
     * @return The value of the homing sensor
     */
    fun getHomingSensor() = homingSensor.get()

    /**
     * Zero the relative encoder
     */
    fun zeroEncoder() {
        relativeEncoder.setPosition(0.0)
    }

    /**
     * Set the speed of the pivot
     * @param speed The speed to set the pivot to
     */
    fun setRawSpeed(speed: Double){
        pivotMotor.set(speed)
    }

    /**
     * Set a voltage amount to send to the pivot
     * @param voltage The voltage to send to the pivot
     */
    fun setVoltage(voltage: Double) {
        pivotMotor.setVoltage(voltage)
    }

    /** Reset the absolute encoder */
    fun resetEncoder() {
        absoluteEncoder.reset()
    }

    /**
     * Set the position of the pivot using PID
     * @param position The position to set the pivot to
     */
    fun setPIDPosition(position: Double) {
        pivotPID.setReference(position, CANSparkBase.ControlType.kPosition)
        println("Setting position to $position")
    }

    /**
     * Set the position of the pivot using PID with feedforward specialized to hold it in place
     * @param position The position to set the pivot to
     */
    fun holdArm(position: Double){
        pivotPID.setReference(position,
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.calculate(Units.degreesToRadians(position), 0.0)
        )
    }

    /** Stop the pivot */
    fun stop() {
        pivotMotor.stopMotor()
    }

}