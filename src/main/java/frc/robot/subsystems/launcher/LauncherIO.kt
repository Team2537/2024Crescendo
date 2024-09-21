package frc.robot.subsystems.launcher

import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface LauncherIO {

    /**
     * The set of data that a launcher module should log.
     */
    data class LauncherInputs(
        private val _position: MutableMeasure<Angle>,
        private val _velocity: MutableMeasure<Velocity<Angle>>,
        private val _applied: MutableMeasure<Voltage>,
        private val _current: MutableMeasure<Current>,
    ) : LoggableInputs {
        // Default value for all will be zero
        // this is fine cause zero is cached anyway,
        // so the garbage immutable measure are not really
        // garbage
        constructor() : this(
            Radians.zero().mutableCopy(),
            RadiansPerSecond.zero().mutableCopy(),
            Volt.zero().mutableCopy(),
            Amps.zero().mutableCopy(),
        )

        // NOTE - *technically* one could get these and then cast
        //  them to `MutableMeasure` to access the `.mut_` methods,
        //  but that possibility is not worth making copies everytime
        //  these are accessed. The solution, of course, would be a
        //  wrapper delegate with a reference to the mutable measure,
        //  but even I have limits on over-engineering for arbitrary
        //  safety standards (which means I might do just that)

        /**
         * The position input
         */
        var position: Measure<Angle>
            get() = _position
            set(value) {
                _position.mut_replace(value)
            }

        /**
         * The velocity input
         */
        var velocity: Measure<Velocity<Angle>>
            get() = _velocity
            set(value) {
                _velocity.mut_replace(value)
            }

        /**
         * The applied voltage to the motor(s)
         */
        var applied: Measure<Voltage>
            get() = _applied
            set(value) {
                _applied.mut_replace(value)
            }

        /**
         * The current going to the motor(s)
         */
        var current: Measure<Current>
            get() = _current
            set(value) {
                _current.mut_replace(value)
            }

        override fun toLog(table: LogTable) {
            table.put("position", position)
            table.put("velocity", velocity)
            table.put("applied", applied)
            table.put("current", current)
        }

        override fun fromLog(table: LogTable) {
            // All of these will default to 0 of their unit
            position = table.get("position", position)
            velocity = table.get("velocity", velocity)
            applied = table.get("applied", applied)
            current = table.get("current", current)
        }
    }

    /**
     * Updates the given inputs with data from this IO layer
     */
    fun updateInputs(inputs: LauncherInputs)

    /**
     * Runs the open loop with the specified voltage.
     *
     * @param voltage The voltage to control the motor(s) with.
     */
    fun runOpen(voltage: Measure<Voltage>)

    /**
     * Runs the closed loop at a specified velocity.
     *
     * @param velocity The velocity to run the motor(s) at.
     */
    fun runClosed(velocity: Measure<Velocity<Angle>>)

    /**
     * Stops the loop.
     */
    fun stop()

    /**
     * Configures the PID controller for the motor(s)
     *
     * @param p The polynomial constant
     * @param i The integral constant
     * @param d The derivative constant
     */
    fun configurePID(p: Double, i: Double, d: Double)

}