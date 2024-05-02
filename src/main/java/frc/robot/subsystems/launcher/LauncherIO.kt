package frc.robot.subsystems.launcher

import com.revrobotics.SparkPIDController.ArbFFUnits
import edu.wpi.first.units.*
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * The IO layer interface for the launcher subsystem.
 * This interface is used to abstract the hardware of the launcher from the rest of the code.
 * This allows for the launcher subsystem to be tested without the hardware present.
 *
 * @see LaunchSubsystem
 * @see LauncherIO.LauncherIOInputs
 * @author Falon Clark
 */
interface LauncherIO {

    /**
     * The inputs for the launcher subsystem.
     * This class is used to store the inputs for the launcher subsystem.
     * This class is used to log the inputs for the launcher subsystem.
     *
     * @see LauncherIO
     * @see LauncherIO.updateInputs
     * @author Falon Clark
     */
    class LauncherIOInputs : LoggableInputs {
        /** Stores the linear velocity of the roller */
        var rollerVelocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.zero(Units.MetersPerSecond)

        /** Stores the position of the roller */
        var rollerPosition: MutableMeasure<Distance> = MutableMeasure.zero(Units.Meters)

        /** Stores the linear velocity of the top flywheels */
        var topFlywheelsVelocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.zero(Units.MetersPerSecond)

        /** Stores the linear velocity of the bottom flywheels */
        var bottomFlywheelsVelocity: MutableMeasure<Velocity<Distance>> = MutableMeasure.zero(Units.MetersPerSecond)

        /** Stores the current applied voltage to the top flywheels */
        var topFlywheelAppliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)

        /** Stores the current applied voltage to the bottom flywheels */
        var bottomFlywheelAppliedVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)

        /** Stores if the right sensor has detected a note */
        var rightNoteDetected: Boolean = false

        /**
         * Puts the values of the inputs into a LogTable.
         * @param table The LogTable to put the values into.
         */
        override fun toLog(table: LogTable?) {
            table?.put("rollerVelocity", rollerVelocity)
            table?.put("flywheelsVelocity_t", topFlywheelsVelocity)
            table?.put("flywheelsVelocity_b", bottomFlywheelsVelocity)
            table?.put("noteDetected_r", rightNoteDetected)
            table?.put("rollerPosition", rollerPosition)
            table?.put("appliedVoltage_t", topFlywheelAppliedVoltage)
            table?.put("appliedVoltage_b", bottomFlywheelAppliedVoltage)
        }

        /**
         * Gets the values of the inputs from a LogTable.
         * @param table The LogTable to get the values from.
         */
        override fun fromLog(table: LogTable?) {
            rollerVelocity.mut_replace(table?.get("rollerVelocity", rollerVelocity))
            topFlywheelsVelocity.mut_replace(table?.get("flywheelsVelocity_t", topFlywheelsVelocity))
            bottomFlywheelsVelocity.mut_replace(table?.get("flywheelsVelocity_b", bottomFlywheelsVelocity))
            rightNoteDetected = table?.get("noteDetected_r", rightNoteDetected) ?: false
            rollerPosition.mut_replace(table?.get("rollerPosition", rollerPosition))
            topFlywheelAppliedVoltage.mut_replace(table?.get("appliedVoltage_t", topFlywheelAppliedVoltage))
            bottomFlywheelAppliedVoltage.mut_replace(table?.get("appliedVoltage_b", bottomFlywheelAppliedVoltage))
        }
    }

    /**
     * Updates the inputs of the launcher subsystem.
     * This method is used to update the inputs of the launcher subsystem.
     * @param inputs The inputs to update to reflect the current state of the launcher
     */
    fun updateInputs(inputs: LauncherIOInputs)

    /**
     * Set a raw percentage power to the roller.
     * @param power The percentage power to set the roller to.
     */
    fun setRollerPower(power: Double)

    /**
     * Set a raw percentage power to the flywheels.
     * @param power The percentage power to set the flywheels to.
     */
    fun setFlywheelPower(power: Double)

    /**
     * Run the flywheels at a set velocity.
     * @param velocity The velocity to run the flywheels at.
     * @param arbFF_t The arbitrary feedforward value for the top flywheels.
     * @param arbFF_b The arbitrary feedforward value for the bottom flywheels.
     * @param arbFFUnits The units of the arbitrary feedforward values.
     */
    fun runSetpoint(velocity: Measure<Velocity<Distance>>, arbFF_t: Double, arbFF_b: Double, arbFFUnits: ArbFFUnits)

    /**
     * Run the roller at a set position.
     * @param position The position to run the roller at.
     */
    fun runRollerSetpoint(position: Measure<Distance>)

    /**
     * Enable or disable the brake mode of the flywheels.
     * @param brake True to enable brake mode, false to disable brake mode.
     */
    fun setFlywheelsBrakeMode(brake: Boolean)

    /**
     * Enable or disable the brake mode of the roller.
     * @param brake True to enable brake mode, false to disable brake mode.
     */
    fun setRollerBrakeMode(brake: Boolean)

    /** Stop the flywheels motion */
    fun stopFlywheels()

    /** Stop the roller motion */
    fun stopRoller()

    /**
     * Send a raw voltage to the flywheels.
     * @param voltage The voltage to send to the flywheels.
     */
    fun setFlywheelVoltage(topVoltage: Measure<Voltage>, bottomVoltage: Measure<Voltage>)

    /**
     * Send a raw voltage to the roller.
     * @param voltage The voltage to send to the roller.
     */
    fun setRollerVoltage(voltage: Measure<Voltage>)
}