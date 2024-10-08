package frc.robot.subsystems.launcher

import edu.wpi.first.units.*
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Robot
import frc.robot.subsystems.pivot.Pivot
import lib.math.units.*
import lib.waitFor
import lib.waitUntil
import lib.waitWhile
import org.littletonrobotics.junction.Logger

// IO is passed in to avoid hard dependency/to decouple.
/**
 * The subsystem that controls the launcher. This will **not** control
 * the pivot arm that the launcher is attached to (see [Pivot]).
 *
 * @author Falon Clark
 * @author Matthew Clark
 * @author Micah Rao
 *
 * @see Pivot
 *
 * @constructor Constructs a launcher subsystem with the specified source of
 * launcher modules to control.
 *
 * @param io The [io layer][LauncherIO] that specifies *what* the subsystem
 * is controlling.
 *
 * @see LauncherIO
 */
class Launcher(private val io: LauncherIO, val flywheelRadius: Measure<Distance> = 2.0 measuredIn Inches) : SubsystemBase() {

    /**
     * [Launcher inputs][LauncherIO.LauncherInputs]
     */
    private val inputs: LauncherIO.LauncherInputs = LauncherIO.LauncherInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Launcher", inputs)

    }

    /**
     * The tangent velocity of the flywheels, as calculated by the
     * flywheel angular velocity and flywheel radius.
     */
    val flywheelLinearVelocity: Measure<Velocity<Distance>>
        get() = angularToLinear(flywheelVelocity, flywheelRadius)

    /**
     * Gets the average velocity of the flywheels.
     *
     * @return The average velocity of the flywheels.
     */
    val flywheelVelocity: Measure<Velocity<Angle>>
        get() {
            return (inputs.topFlywheel.velocity + inputs.bottomFlywheel.velocity) * 0.5
        }

    /**
     * Checks whether the note is being detected
     * @return Whether the note is being detected
     */
    val noteDetected: Boolean get() = inputs.hasNote

    /**
     * Gets a [Trigger] that is `true` when the launcher has a note,
     * and `false` otherwise.
     *
     * @return A trigger for the note detection.
     */
    val noteTrigger: Trigger by lazy {
        Trigger {
            inputs.hasNote && Robot.isEnabled
        }.debounce(0.1)
    }

    /**
     * Sets the flywheel speeds to a velocity
     * @param velocity The velocity to set the flywheels to
     */
    fun setFlywheelVelocity(velocity: RotationVelocity){
        io.setFlywheelVelocity(velocity)
    }

    /**
     * Sets the flywheel speeds to a voltage
     * @param voltage The voltage to set the flywheels to
     */
    fun setFlywheelVoltage(voltage: Measure<Voltage>){
        io.setFlywheelVoltage(voltage)
    }

    /**
     * Stops the flywheels.
     */
    fun stopFlywheels() {
        io.stopFlywheels()
    }

    /**
     * Stops the roller.
     */
    fun stopRoller() {
        io.stopRoller()
    }

    /**
     * Sets the roller target position to a setpoint
     * @param position The position to set the target to
     */
    fun setRollerPosition(position: Measure<Angle>) {
        io.setRollerPosition(position)
    }

    /**
     * Gets the current position of the roller
     * @return The current position of the roller
     */
    fun getRollerPosition(): Measure<Angle> {
        return inputs.rollerRelativePosition.view
    }

    /**
     * The position of the roller. Semantically, the individual
     * get and set methods are more "correct."
     *
     * @see getRollerPosition
     * @see setRollerPosition
     */
    var rollerPosition: Measure<Angle>
        get() = getRollerPosition()
        set(value) = setRollerPosition(value)

    /**
     * Powers the roller's motor to rotate at the given velocity.
     *
     * @param velocity The desired velocity.
     */
    fun setRollerVelocity(velocity: Measure<Velocity<Angle>>) {
        io.setRollerVelocity(velocity)
    }

    /**
     * Gets the roller's current velocity.
     *
     * @return The roller's velocity
     */
    fun getRollerVelocity(): Measure<Velocity<Angle>> {
        return inputs.rollerVelocity
    }

    /**
     * The velocity of the roller. Semantically, the individual
     * get and set methods are more "correct."
     *
     * @see getRollerVelocity
     * @see setRollerVelocity
     */
    var rollerVelocity: Measure<Velocity<Angle>>
        get() = getRollerVelocity()
        set(value) = setRollerVelocity(value)

    /**
     * Gets a command that, when run, will power the flywheels and feed the note
     * into them. If no note is present when the command is run, nothing happens.
     *
     * @param desiredVelocity The desired linear velocity of the note as it
     * exits the launcher.
     *
     * @return A command that launches a note.
     */
    fun getLaunchCommand(desiredVelocity: Measure<Velocity<Distance>>): Command {
        val desiredFlywheelVelocity = linearToAngular(desiredVelocity, flywheelRadius)
        val errorTolerance = 0.05 // percent

        return Commands.either(
            // Only run if we have a note.
            Commands.sequence(
                runOnce { io.setFlywheelLinearVelocity(desiredVelocity) },

                waitUntil {
                    inputs.topFlywheel.velocity.isNear(desiredFlywheelVelocity, errorTolerance) &&
                            inputs.bottomFlywheel.velocity.isNear(desiredFlywheelVelocity, errorTolerance)
                },

                runOnce { io.setRollerVelocity(desiredFlywheelVelocity) }, // speed is key

                waitWhile(inputs::hasNote),

                runOnce {
                    stopFlywheels()
                    stopRoller()
                },
            ),
            Commands.none(),
            inputs::hasNote,
        )
    }

    /**
     * Gets a command that, when run, will power the flywheels and feed the note
     * to them. If no note is present when the command is run, nothing happens.
     *
     * @param desiredFlywheelVelocity The velocity to get the flywheels at before
     * feeding them the note. Defaults to 6000 rpm.
     *
     * @return A command that launches a note.
     */
    fun getLaunchCommand(desiredFlywheelVelocity: Measure<Velocity<Angle>> = 6000.rpm): Command {
        val errorTolerance = 0.05 // percent

        return Commands.either(
            // Only run if we have a note.
            Commands.sequence(
                runOnce { io.setFlywheelVelocity(desiredFlywheelVelocity) },

                waitUntil {
                    inputs.topFlywheel.velocity.isNear(desiredFlywheelVelocity, errorTolerance) &&
                    inputs.bottomFlywheel.velocity.isNear(desiredFlywheelVelocity, errorTolerance)
                },

                runOnce { io.setRollerVelocity(desiredFlywheelVelocity) }, // speed is key

                waitWhile(inputs::hasNote),

                runOnce {
                    stopFlywheels()
                    stopRoller()
                },
            ),
            Commands.none(),
            inputs::hasNote,
        )
    }

    /**
     * Gets a command that, when run, will attempt to load a note from
     * the intake into the launcher.
     *
     * @return A command to load notes from the intake into the launcher.
     */
    fun getLoadCommand(): Command {
        return Commands.sequence(
            runOnce {
                // Avoid launching the note prematurely
                stopFlywheels()

                // FIXME: use a real roller velocity
                rollerVelocity = 20.rpm
            },

            waitUntil(inputs::hasNote),
            // The old command waited for 0.15 seconds after detecting
            // the note, so I'm doing that here as well.
            waitFor(0.15),
        )
    }

    /**
     * Gets a command that stops the motors.
     *
     * @return A command that stops the launcher motors.
     */
    fun getStopCommand(): Command {
        return runOnce {
            stopFlywheels()
            stopRoller()
        }
    }
}