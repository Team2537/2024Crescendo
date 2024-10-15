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

        /** Container class for flywheels */
        class FlywheelInputs(private val isTop: Boolean) : LoggableInputs {

            // Flywheel
            /** The position of the flywheel motor(s), measured relatively by their encoder(s). */
            @JvmField
            val relativePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

//        /** The position of the flywheel motor(s), measured absolutely. */
//        @JvmField
//        val flywheelAbsolutePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

            /** The velocity of the flywheel motor(s)'s rotation. */
            @JvmField
            val velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(RotationsPerSecond)

            /** The voltage applied to the flywheel motor(s). */
            @JvmField
            val appliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Volts)

            /** The voltage applied to the flywheel motor(s). */
            @JvmField
            val appliedCurrent: MutableMeasure<Current> = MutableMeasure.zero(Amps)

            override fun toLog(table: LogTable) {
                val prefix = if(isTop) "top" else "bottom"
                table.put("${prefix}FlywheelRelativePosition", relativePosition)
//            table.put("${prefix}flywheelAbsolutePosition", flywheelAbsolutePosition)
                table.put("${prefix}FlywheelVelocity", velocity)
                table.put("${prefix}FlywheelAppliedVoltage", appliedVoltage)
                table.put("${prefix}FlywheelAppliedCurrent", appliedCurrent)
            }

            override fun fromLog(table: LogTable) {
                val prefix = if(isTop) "top" else "bottom"
                relativePosition.mut_replace(table.get("${prefix}FlywheelRelativePosition", relativePosition))
    //            flywheelAbsolutePosition.mut_replace(table.get("flywheelAbsolutePosition", flywheelAbsolutePosition))
                velocity.mut_replace(table.get("${prefix}FlywheelVelocity", velocity))
                appliedVoltage.mut_replace(table.get("${prefix}FlywheelAppliedVoltage", appliedVoltage))
                appliedCurrent.mut_replace(table.get("${prefix}FlywheelAppliedCurrent", appliedCurrent))
            }
        }

        /** Container for the top flywheel's inputs */
        val topFlywheel = FlywheelInputs(true)

        /** Container for the bottom flywheel's inputs */
        val bottomFlywheel = FlywheelInputs(false)

        // Roller
        /** The position of the flywheel motor(s), measured relatively by their encoder(s). */
        @JvmField
        val rollerRelativePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

//        /** The position of the flywheel motor(s), measured absolutely. */
//        @JvmField
//        val rollerAbsolutePosition: MutableMeasure<Angle> = MutableMeasure.zero(Rotations)

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

        override fun toLog(table: LogTable) {
            topFlywheel.toLog(table)
            bottomFlywheel.toLog(table)

            table.put("rollerRelativePosition", rollerRelativePosition)
//            table.put("rollerAbsolutePosition", rollerAbsolutePosition)
            table.put("rollerVelocity", rollerVelocity)
            table.put("rollerAppliedVoltage", rollerAppliedVoltage)
            table.put("rollerAppliedCurrent", rollerAppliedCurrent)

            table.put("hasNote", hasNote)
        }

        override fun fromLog(table: LogTable) {
            topFlywheel.fromLog(table)
            bottomFlywheel.fromLog(table)

            rollerRelativePosition.mut_replace(table.get("rollerRelativePosition", rollerRelativePosition))
//            rollerAbsolutePosition.mut_replace(table.get("rollerAbsolutePosition", rollerAbsolutePosition))
            rollerVelocity.mut_replace(table.get("rollerVelocity", rollerVelocity))
            rollerAppliedVoltage.mut_replace(table.get("rollerAppliedVoltage", rollerAppliedVoltage))
            rollerAppliedCurrent.mut_replace(table.get("rollerAppliedCurrent", rollerAppliedCurrent))

            hasNote = table.get("hasNote", hasNote)
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
    fun setFlywheelVoltage(voltage: Measure<Voltage>) {
        setFlywheelVoltage(voltage, voltage)
    }

    /**
     * Runs the flywheels with the specified voltages.
     *
     * @param top The voltage for the top flywheels.
     * @param bottom The voltage for the bottom flywheels.
     */
    fun setFlywheelVoltage(top: Measure<Voltage>, bottom: Measure<Voltage>) {
        setTopFlywheelVoltage(top)
        setBottomFlywheelVoltage(bottom)
    }

    /**
     * Runs the top flywheels with the specified voltage.
     *
     * @param voltage The voltage to control the flywheel motor(s) with.
     */
    fun setTopFlywheelVoltage(voltage: Measure<Voltage>) {}

    /**
     * Runs the bottom flywheels with the specified voltage.
     *
     * @param voltage The voltage to control the flywheel motor(s) with.
     */
    fun setBottomFlywheelVoltage(voltage: Measure<Voltage>) {}

    /**
     * Runs the flywheels so that they may launch the note at the
     * given exit velocity.
     *
     * @param velocity The desired exit velocity of the note.
     */
    fun setFlywheelLinearVelocity(velocity: Measure<Velocity<Distance>>) {}

    /**
     * Runs the flywheels at a specified velocity.
     *
     * @param velocity The velocity to run the flywheel motor(s) at.
     */
    fun setFlywheelVelocity(velocity: Measure<Velocity<Angle>>) {}

    /**
     * Runs the flywheels at a specified velocity.
     *
     * @param top The velocity for the top flywheel.
     * @param bottom The velocity for the bottom flywheel.
     */
    fun setFlywheelVelocity(top: Measure<Velocity<Angle>>, bottom: Measure<Velocity<Angle>>) {}

    /**
     * Runs the top flywheels at a specified velocity.
     *
     * @param velocity The velocity to run the flywheel motor(s) at.
     */
    fun setTopFlywheelVelocity(velocity: Measure<Velocity<Angle>>) {}

    /**
     * Runs the bottom flywheels at a specified velocity.
     *
     * @param velocity The velocity to run the flywheel motor(s) at.
     */
    fun setBottomFlywheelVelocity(velocity: Measure<Velocity<Angle>>) {}

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
     * Moves the roller to a specified position.
     *
     * @param position The desired position.
     */
    fun setRollerPosition(position: Measure<Angle>) {}

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

    // Stop probably shouldn't be separated
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
    fun setFlywheelPID(p: Double, i: Double, d: Double) {
        setTopFlywheelPID(p, i, d)
        setBottomFlywheelPID(p, i, d)
    }

    /**
     * Configures the PID controller for the top flywheel.
     *
     * @param p The proportional gain
     * @param i The integral gain
     * @param d The derivative gain
     */
    fun setTopFlywheelPID(p: Double, i: Double, d: Double) {}

    /**
     * Configures the PID controller for the bottom flywheel.
     *
     * @param p The proportional gain
     * @param i The integral gain
     * @param d The derivative gain
     */
    fun setBottomFlywheelPID(p: Double, i: Double, d: Double) {}

    /**
     * Configures the feed forward for the flywheel motor(s)
     *
     * @param s The static gain.
     * @param v The velocity gain.
     * @param a The acceleration gain.
     */
    fun setFlywheelFeedForward(s: Double, v: Double, a: Double) {
        setTopFlywheelFeedForward(s, v, a)
        setBottomFlywheelFeedForward(s, v, a)
    }

    /**
     * Configures the feed forward for the top flywheel.
     *
     * @param s The static gain.
     * @param v The velocity gain.
     * @param a The acceleration gain.
     */
    fun setTopFlywheelFeedForward(s: Double, v: Double, a: Double) {}

    /**
     * Configures the feed forward for the bottom flywheel.
     *
     * @param s The static gain.
     * @param v The velocity gain.
     * @param a The acceleration gain.
     */
    fun setBottomFlywheelFeedForward(s: Double, v: Double, a: Double) {}

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

    fun setBrakes(topBrake: Boolean, bottomBrake: Boolean, rollerBrake: Boolean) {}
}