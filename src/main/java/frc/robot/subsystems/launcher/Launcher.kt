package frc.robot.subsystems.launcher

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
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
import lib.math.units.RotationVelocity
import lib.math.units.meters
import lib.math.units.view
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
class Launcher(private val io: LauncherIO) : SubsystemBase() {

    // I prefer decoupling the io from the subsystem to avoid hard dependencies,
    // which I know is not the convention set by Falon's subsystems. If this is
    // not preferable to a hard-coded when statement, it's fairly easy to change
    // This would just be the primary constructor.
    constructor() : this(
        when (Constants.RobotConstants.mode) {
            Constants.RobotConstants.Mode.REAL -> LauncherIONeos(
                Constants.LauncherConstants.TOP_FLYWHEELS,
                Constants.LauncherConstants.BOTTOM_FLYWHEELS,
                Constants.LauncherConstants.ROLLER_MOTOR,
                Constants.LauncherConstants.RIGHT_NOTE_DETECTOR,
                Double.NaN.meters, // FIXME: actual flywheel radius
            )

            Constants.RobotConstants.Mode.SIM -> TODO()
            Constants.RobotConstants.Mode.REPLAY -> TODO()
        }
    )

    private val inputs: LauncherIO.LauncherInputs = LauncherIO.LauncherInputs()

//    /** Shuffleboard tab for the frc.robot.subsystems.launcher.Launcher subsystem */
//    private val tab = Shuffleboard.getTab("frc.robot.subsystems.launcher.Launcher")

//    /** Trigger for when the note is detected */
//    private val noteTrigger: Trigger

    /**
     * The system identification routine for the launcher subsystem.
     */
    private val routine: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage> ->
//                topFlywheels.setVoltage(volts.into(Units.Volts))
                io.setFlywheelVoltage(volts)
            },
            { log: SysIdRoutineLog ->
//                log.motor("topFlywheels")
//                    .voltage(Units.Volts.of(topFlywheels.appliedOutput * topFlywheels.busVoltage))
//                    .angularPosition(Units.Rotations.of(topFlywheels.encoder.position))
//                    .angularVelocity(Units.RPM.of(topFlywheels.encoder.velocity))
//                    .current(Units.Amps.of(topFlywheels.outputCurrent))
//                log.motor("bottomFlywheels")
//                    .voltage(Units.Volts.of(bottomFlywheels.appliedOutput * bottomFlywheels.busVoltage))
//                    .angularPosition(Units.Rotations.of(bottomFlywheels.encoder.position))
//                    .angularVelocity(Units.RPM.of(bottomFlywheels.encoder.velocity))
//                    .current(Units.Amps.of(bottomFlywheels.outputCurrent))
                // I think?
                // Idk if this is being completely replaced by advkit
                // I assume it is, but I won't remove it 'till I know for sure.
                log.motor("topFlywheels")
                    .voltage(inputs.topFlywheel.appliedVoltage)
                    .angularPosition(inputs.topFlywheel.relativePosition)
                    .angularVelocity(inputs.topFlywheel.velocity)
                    .current(inputs.topFlywheel.appliedCurrent)
            },
            this
        )
    )

//    /** Feedforward controller for the top flywheel */
//    val topFlywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(-0.20832, 0.11109, 0.024896)
//    /** Feedforward controller for the bottom flywheel */
//    val bottomFlywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.035079, 0.10631, 0.0080339)

//    init {
        // So much logging oh my god I forgot there was so much logging
//        Shuffleboard.getTab("frc.robot.subsystems.launcher.Launcher").addBoolean("Note Detected") { noteTrigger.asBoolean }
//        Shuffleboard.getTab("frc.robot.subsystems.launcher.Launcher").addDouble("Position") { getRollerPosition() }
//        Shuffleboard.getTab("frc.robot.subsystems.launcher.Launcher").addDouble("Setpoint") { setPoint }
//        Shuffleboard.getTab("frc.robot.subsystems.launcher.Launcher Sysid").addDouble("Top Flywheel Position") { topFlywheels.encoder.position }
//        Shuffleboard.getTab("frc.robot.subsystems.launcher.Launcher Sysid").addDouble("Top Flywheel Velocity") { topFlywheels.encoder.velocity }
//        Shuffleboard.getTab("frc.robot.subsystems.launcher.Launcher Sysid").addDouble("Bottom Flywheel Position") { bottomFlywheels.encoder.position }
//        Shuffleboard.getTab("frc.robot.subsystems.launcher.Launcher Sysid").addDouble("Bottom Flywheel Velocity") { bottomFlywheels.encoder.velocity }
//        Shuffleboard.getTab("frc.robot.subsystems.launcher.Launcher Sysid").addDouble("Top Flywheel Voltage") {topFlywheels.appliedOutput * topFlywheels.busVoltage }
//        Shuffleboard.getTab("frc.robot.subsystems.launcher.Launcher Sysid").addDouble("Bottom Flywheel Voltage") {bottomFlywheels.appliedOutput * bottomFlywheels.busVoltage }
//    }

//    /**
//     * Set the flywheel speeds to a raw duty cycle value
//     * @param rawSpeed The raw duty cycle value to set the flywheels to
//     */
//    fun setFlywheelSpeeds(rawSpeed: Double) {
//        topFlywheels.set(rawSpeed)
//        bottomFlywheels.set(rawSpeed)
//    }
//
//    /**
//     * Set the roller speed to a raw duty cycle value
//     * @param rawSpeed The raw duty cycle value to set the roller to
//     */
//    fun setRollerSpeed(rawSpeed: Double) {
//        rollerMotor.set(rawSpeed)
//    }

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

    /**
     * Creates a command to run a dynamic system identification routine
     * @param direction The direction to run the routine in
     * @return The command to run the routine
     */
    fun dynamicSysIDRoutine(direction: Direction): Command? {
        return routine.dynamic(direction)
    }

    /**
     * Creates a command to run a quasi-static system identification routine
     * @param direction The direction to run the routine in
     * @return The command to run the routine
     */
    fun quasiStaticSysIDRoutine(direction: Direction): Command? {
        return routine.quasistatic(direction)
    }
}