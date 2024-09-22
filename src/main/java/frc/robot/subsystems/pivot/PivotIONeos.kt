package frc.robot.subsystems.pivot

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import lib.math.units.into

class PivotIONeos(
    private val pivotID: Int,
    private val pivotInverted: Boolean,
    private val absoluteEncoderID: Int,
    private val rotorToArmRatio: Double,
    private val encoderToArmRatio: Double,
    private val homingSensorID: Int,
    private val kP: Double,
    private val kI: Double,
    private val kD: Double,
    private val kS: Double,
    private val kG: Double,
    private val kV: Double,
    private val kA: Double
) : PivotIO {

    /** The motor for the pivot */
    private val pivotMotor: CANSparkMax = CANSparkMax(pivotID, CANSparkLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        inverted = pivotInverted
        encoder.positionConversionFactor = rotorToArmRatio
        encoder.velocityConversionFactor = rotorToArmRatio

        pidController.setP(kP)
        pidController.setI(kI)
        pidController.setD(kD)

        setSmartCurrentLimit(40)
    }

    /** The absolute encoder for the pivot */
    private val absoluteEncoder: DutyCycleEncoder = DutyCycleEncoder(absoluteEncoderID).apply {
        distancePerRotation = encoderToArmRatio
    }

    /** Feedforward controller for the pivot */
    private val feedforward: ArmFeedforward = ArmFeedforward(kS, kG, kV, kA)

    /** The homing sensor for the pivot, to trigger when the pivot is upright */
    private val homingSensor: DigitalInput = DigitalInput(homingSensorID)

    /**
     * Updates the inputs with new data.
     * @param inputs The data to update with the new sensor information, mutated in place.
     */
    override fun updateInputs(inputs: PivotIO.PivotInputs) {
        inputs.isAtHardstop = homingSensor.get()
        inputs.relativePosition.mut_replace(pivotMotor.encoder.position, Units.Rotations)
        inputs.absolutePosition.mut_replace(absoluteEncoder.distance, Units.Rotations)
        inputs.velocity.mut_replace(pivotMotor.encoder.velocity, Units.RotationsPerSecond)
        inputs.appliedVoltage.mut_replace(pivotMotor.appliedOutput, Units.Volts)
        inputs.appliedCurrent.mut_replace(pivotMotor.outputCurrent, Units.Amps)
    }

    /**
     * Sets the raw voltage applied to the motor(s).
     * @param voltage The voltage to apply.
     * @param isPID Whether the voltage is being set by a PID controller. (Used for simulation purposes.)
     */
    override fun setRawVoltage(voltage: Measure<Voltage>, isPID: Boolean) {
        pivotMotor.setVoltage(voltage into Units.Volts)
    }

    /**
     * Reset the relative encoder to a known position.
     * @param position The known position to reset to.
     */
    override fun setKnownPosition(position: Measure<Angle>) {
        pivotMotor.encoder.position = position into Units.Rotations
    }

    /**
     * Set the target position of the pivot.
     * @param position The target position to set.
     */
    override fun setTargetPosition(position: Measure<Angle>) {
        pivotMotor.pidController.setReference(
            position into Units.Rotations,
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward.calculate(position into Units.Radians, 0.0)
        )
    }

    /**
     * Set the PID gains of the pivot.
     * @param p The proportional gain.
     * @param i The integral gain.
     * @param d The derivative gain.
     */
    override fun setPID(p: Double, i: Double, d: Double) {
        pivotMotor.pidController.p = p
        pivotMotor.pidController.i = i
        pivotMotor.pidController.d = d
    }
}