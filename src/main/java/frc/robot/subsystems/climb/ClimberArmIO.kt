package frc.robot.subsystems.climb

import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * An IO layer for interacting with the robot's climb systems
 *
 * @see Climb
 */
interface ClimberArmIO {
    class ClimberArmInputs : LoggableInputs {
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
            table.put("motorRelativePosition", relativePosition)
            table.put("motorVelocity", velocity)
            table.put("motorAppliedVoltage", appliedVoltage)
            table.put("motorAppliedCurrent", appliedCurrent)
        }

        override fun fromLog(table: LogTable) {
            relativePosition.mut_replace(table.get("motorRelativePosition", relativePosition))
            velocity.mut_replace(table.get("motorVelocity", velocity))
            appliedVoltage.mut_replace(table.get("motorAppliedVoltage", appliedVoltage))
            appliedCurrent.mut_replace(table.get("motorAppliedCurrent", appliedCurrent))
        }
    }

    /**
     * Updates the [loggable inputs](ClimberArmInputs) with data from this IO layer.
     *
     * @param inputs The inputs to update.
     *
     * @see ClimberArmInputs
     */
    fun updateInputs(inputs: ClimberArmInputs) {}

    /**
     * Supplies the given voltage to the motors.
     *
     * @param voltage The voltage to supply
     */
    fun setVoltage(voltage: Measure<Voltage>, isClosedLoop: Boolean = false)
}