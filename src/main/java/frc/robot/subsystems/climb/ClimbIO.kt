package frc.robot.subsystems.climb

import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import lib.math.units.RotationVelocity
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * An IO layer for interacting with the robot's climb systems
 *
 * @see Climb
 */
interface ClimbIO {
    class ClimbInputs : LoggableInputs {
        class MotorInputs(private inline val prefix: String) : LoggableInputs {
            /**
             * The relative position as measured by the motor's encoder.
             */
            val relativePosition: MutableMeasure<Angle> = MutableMeasure.zero(Radians)

            /**
             * The angular velocity as measured by the motor's encoder.
             */
            val velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RadiansPerSecond)

            /**
             * The voltage applied to the motor.
             */
            val appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

            /**
             * The current applied to the motor.
             */
            val appliedCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)

            override fun toLog(table: LogTable) {
                table.put("${prefix}MotorRelativePosition", relativePosition)
                table.put("${prefix}MotorVelocity", velocity)
                table.put("${prefix}MotorAppliedVoltage", appliedVoltage)
                table.put("${prefix}MotorAppliedCurrent", appliedCurrent)
            }

            override fun fromLog(table: LogTable) {
                relativePosition.mut_replace(table.get("${prefix}MotorRelativePosition", relativePosition))
                velocity.mut_replace(table.get("${prefix}MotorVelocity", velocity))
                appliedVoltage.mut_replace(table.get("${prefix}MotorAppliedVoltage", appliedVoltage))
                appliedCurrent.mut_replace(table.get("${prefix}MotorAppliedCurrent", appliedCurrent))
            }
        }

        /**
         * The left motor's inputs
         */
        val leftMotor: MotorInputs = MotorInputs("left")

        /**
         * The right motor's inputs
         */
        val rightMotor: MotorInputs = MotorInputs("right")

        override fun toLog(table: LogTable) {
            leftMotor.toLog(table)
            rightMotor.toLog(table)
        }

        override fun fromLog(table: LogTable) {
            leftMotor.fromLog(table)
            rightMotor.fromLog(table)
        }
    }

    /**
     * Updates the [loggable inputs](ClimbInputs) with data from this IO layer.
     *
     * @param inputs The inputs to update.
     *
     * @see ClimbInputs
     */
    fun updateInputs(inputs: ClimbInputs) {}

    /**
     * Supplies the given voltage to the motors.
     *
     * @param voltage The voltage to supply
     */
    fun setVoltage(voltage: Measure<Voltage>)

    /**
     * Attempts to run the motors at the given angular velocity.
     *
     * @param velocity The velocity at which the motors should spin
     */
    fun setMotorVelocity(velocity: Measure<Velocity<Angle>>)

    /**
     * Attempts to run the motors so that the robot would ascend/descend
     * with the given linear velocity. The method will assume that only
     * gearing ratios affect the conversion from angular to linear velocity.
     *
     * @param velocity The velocity to climb at.
     */
    fun setLinearVelocity(velocity: Measure<Velocity<Distance>>)
}