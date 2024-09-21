package frc.robot.subsystems.launcher

import edu.wpi.first.units.*
import edu.wpi.first.units.Units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * An IO layer for the launcher module that can log with AdvantageKit.
 */
interface LauncherIO {

    /**
     * The set of data that a launcher module should log.
     */
     class LauncherInputs : LoggableInputs {

        // Flywheel
        /** The position of the flywheel motor(s), measured relatively by their encoder(s). */
        @JvmField
        val flywheelRelativePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

        /** The position of the flywheel motor(s), measured absolutely. */
        @JvmField
        val flywheelAbsolutePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

        /** The velocity of the flywheel motor(s)'s rotation. */
        @JvmField
        val flywheelVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RotationsPerSecond)

        /** The voltage applied to the flywheel motor(s). */
        @JvmField
        val flywheelAppliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

        /** The voltage applied to the flywheel motor(s). */
        @JvmField
        val flywheelAppliedCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)

        // Roller
        /** The position of the flywheel motor(s), measured relatively by their encoder(s). */
        @JvmField
        val rollerRelativePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

        /** The position of the flywheel motor(s), measured absolutely. */
        @JvmField
        val rollerAbsolutePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

        /** The velocity of the flywheel motor(s)'s rotation. */
        @JvmField
        val rollerVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RotationsPerSecond)

        /** The voltage applied to the flywheel motor(s). */
        @JvmField
        val rollerAppliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

        /** The voltage applied to the flywheel motor(s). */
        @JvmField
        val rollerAppliedCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)

        /** Whether the launcher is holding a note or not. */
        @JvmField
        var hasNote: Boolean = false

        /** Whether the launcher is ready/able to shoot. */
        @JvmField
        var canShoot: Boolean = false

        override fun toLog(table: LogTable) {
            // NOTE - I actually am not sure on if name mangling is needed
            table.put("flywheelRelativePosition", flywheelRelativePosition)
            table.put("flywheelAbsolutePosition", flywheelAbsolutePosition)
            table.put("flywheelVelocity", flywheelVelocity)
            table.put("flywheelAppliedVoltage", flywheelAppliedVoltage)
            table.put("flywheelAppliedCurrent", flywheelAppliedCurrent)

            table.put("rollerRelativePosition", rollerRelativePosition)
            table.put("rollerAbsolutePosition", rollerAbsolutePosition)
            table.put("rollerVelocity", rollerVelocity)
            table.put("rollerAppliedVoltage", rollerAppliedVoltage)
            table.put("rollerAppliedCurrent", rollerAppliedCurrent)

            table.put("hasNote", hasNote)
            table.put("canShoot", canShoot)
        }

        override fun fromLog(table: LogTable) {
            flywheelRelativePosition.mut_replace(table.get("flywheelRelativePosition", flywheelRelativePosition))
            flywheelAbsolutePosition.mut_replace(table.get("flywheelAbsolutePosition", flywheelAbsolutePosition))
            flywheelVelocity.mut_replace(table.get("flywheelVelocity", flywheelVelocity))
            flywheelAppliedVoltage.mut_replace(table.get("flywheelAppliedVoltage", flywheelAppliedVoltage))
            flywheelAppliedCurrent.mut_replace(table.get("flywheelAppliedCurrent", flywheelAppliedCurrent))

            rollerRelativePosition.mut_replace(table.get("rollerRelativePosition", rollerRelativePosition))
            rollerAbsolutePosition.mut_replace(table.get("rollerAbsolutePosition", rollerAbsolutePosition))
            rollerVelocity.mut_replace(table.get("rollerVelocity", rollerVelocity))
            rollerAppliedVoltage.mut_replace(table.get("rollerAppliedVoltage", rollerAppliedVoltage))
            rollerAppliedCurrent.mut_replace(table.get("rollerAppliedCurrent", rollerAppliedCurrent))

            hasNote = table.get("hasNote", hasNote)
            canShoot = table.get("canShoot", canShoot)
        }
    }

    /**
     * Updates the given inputs with data from this IO layer
     */
    fun updateInputs(inputs: LauncherInputs) {}

    /**
     * Runs the flywheels with the specified voltage.
     *
     * @param voltage The voltage to control the flywheel motor(s) with.
     */
    fun setFlywheelVoltage(voltage: Measure<Voltage>) {}

    /**
     * Runs the flywheels at a specified velocity.
     *
     * @param velocity The velocity to run the flywheel motor(s) at.
     */
    fun setFlywheelVelocity(velocity: Measure<Velocity<Angle>>) {}

    /**
     * Runs the roller with the specified voltage.
     *
     * @param voltage The voltage to control the roller motor(s) with.
     */
    fun setRollerVoltage(voltage: Measure<Voltage>) {}

    /**
     * Runs the roller at a specified velocity.
     *
     * @param velocity The velocity to run the roller motor(s) at.
     */
    fun setRollerVelocity(velocity: Measure<Velocity<Angle>>) {}

    /**
     * Stops the motor(s).
     */
    fun stop() {
        stopFlywheels()
        stopRoller()
    }

    /**
     * Stops the roller.
     */
    fun stopRoller() {}

    /**
     * Stops the flywheels
     */
    fun stopFlywheels() {}

    /**
     * Configures the PID controller for the flywheel motor(s)
     *
     * @param p The proportional gain
     * @param i The integral gain
     * @param d The derivative gain
     */
    fun setFlywheelPID(p: Double, i: Double, d: Double) {}

    /**
     * Configures the feed forward for the flywheel motor(s)
     *
     * @param s The static gain.
     * @param v The velocity gain.
     * @param a The acceleration gain.
     */
    fun setFlywheelFeedForward(s: Double, v: Double, a: Double) {}

    /**
     * Configures the PID controller for the roller motor(s)
     *
     * @param p The proportional gain
     * @param i The integral gain
     * @param d The derivative gain
     */
    fun setRollerPID(p: Double, i: Double, d: Double) {}

    /**
     * Configures the feed forward for the roller motor(s)
     *
     * @param s The static gain.
     * @param v The velocity gain.
     * @param a The acceleration gain.
     */
    fun setRollerFeedForward(s: Double, v: Double, a: Double) {}
}