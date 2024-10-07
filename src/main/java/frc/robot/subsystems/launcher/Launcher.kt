package frc.robot.subsystems.launcher

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.subsystems.pivot.Pivot
import lib.math.units.*
import org.littletonrobotics.junction.Logger
import kotlin.Double.Companion.NaN

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
class Launcher(private val io: LauncherIO) : SubsystemBase() {

    private val inputs: LauncherIO.LauncherInputs = LauncherIO.LauncherInputs()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Launcher", inputs)

    }

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
     * Checks whether the note is being detected
     * @return Whether the note is being detected
     */
    val noteDetected: Boolean get() = inputs.hasNote
}