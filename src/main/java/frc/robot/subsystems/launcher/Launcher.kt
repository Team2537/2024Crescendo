package frc.robot.subsystems.launcher

import edu.wpi.first.units.*
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.Robot
import frc.robot.subsystems.pivot.Pivot
import lib.math.units.*
import lib.waitUntil
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
class Launcher(private val io: LauncherIO, val flywheelRadius: Measure<Distance> = 3.0 measuredIn Inches) : SubsystemBase() {

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
        get() = angVelToLinVel(flywheelVelocity, flywheelRadius)

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
     * Gets a command that, when run, will power the flywheels and feed the note
     * into them. If no note is present when the command is run, nothing happens.
     *
     * @param desiredVelocity The desired angular velocity of the flywheels as
     * they launch the note.
     *
     * @param percentTolerance The percent tolerance for the flywheel velocity to be considered at the desired velocity.
     *
     * @return A command that launches a note.
     */
    fun getLaunchCommand(desiredVelocity: Measure<Velocity<Angle>>, percentTolerance: Double = 0.05): Command {
        return Commands.sequence(
            runOnce {
                io.setBrakes(
                    topBrake = false,
                    bottomBrake = false,
                    rollerBrake = true
                )
                io.setFlywheelVelocity(desiredVelocity)
            },
            waitUntil { inputs.topFlywheel.velocity.isNear(desiredVelocity, percentTolerance) && inputs.bottomFlywheel.velocity.isNear(desiredVelocity, percentTolerance) },
            runOnce { io.setRollerVoltage(12 measuredIn Volts)},
            Commands.waitUntil { !inputs.hasNote },
            Commands.waitSeconds(0.5),
            runOnce {
                io.setRollerVoltage(0 measuredIn Volts)
                io.setFlywheelVelocity(0.rpm)
                io.setBrakes(
                    topBrake = true,
                    bottomBrake = true,
                    rollerBrake = true
                )
            }
        ).onlyIf { inputs.hasNote }
    }

    /**
     * Gets a command that, when run, will power the flywheels and feed the note
     * into them. If no note is present when the command is run, nothing happens.
     *
     * @param desiredVelocity The desired linear velocity of the note as it is launched.
     *
     * @param percentTolerance The percent tolerance for the flywheel velocity to be considered at the desired velocity.
     *
     * @return A command that launches a note.
     */
    fun getLaunchCommand(desiredVelocity: Measure<Velocity<Distance>>, percentTolerance: Double = 0.05): Command {
        return getLaunchCommand(linVelToAngVel(desiredVelocity, flywheelRadius), percentTolerance)
    }

    /**
     * Gets a command that, when run, will attempt to load a note from
     * the intake into the launcher.
     *
     * @return A command to load notes from the intake into the launcher.
     */
    fun getLoadCommand(): Command {
        return Commands.sequence(
            runOnce { io.setRollerPosition(inputs.rollerRelativePosition + (linPosToAngPos(5.0 measuredIn Inches, flywheelRadius))) },
            Commands.waitUntil(noteTrigger)
        )
    }

    /**
     * Gets a command that stops the motors.
     *
     * @return A command that stops the launcher motors.
     */
    fun getStopCommand(): Command = runOnce { io.stop() }
}