package frc.robot.commands.swerve

import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.util.SingletonXboxController
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

/**
 * A Command that allows rotation around a corner/swerve module.
 * @param rotation Double supplier that returns the desired angular velocity of the robot.
 * @param driveMode Boolean supplier that returns true if the robot should drive in field-oriented mode.
 * @param slowMode Boolean supplier that returns true if the robot should drive in slow mode.
 * @see SwerveSubsystem
 */
class CornerSpinCommand(
    rotation: DoubleSupplier,
    driveMode: BooleanSupplier,
    slowMode: BooleanSupplier,
) : Command() {
    private val swerveSubsystem = SwerveSubsystem

    private val rotation: DoubleSupplier

    private val driveMode: BooleanSupplier
    private val slowMode: BooleanSupplier

    private val controller: SwerveController

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
        this.rotation = rotation
        this.driveMode = driveMode
        this.slowMode = slowMode

        controller = swerveSubsystem.getSwerveController()
    }

    /** @suppress */
    override fun initialize() {}

    /** @suppress */
    override fun execute() {
        var rot = rotation.asDouble
        val driveMode = driveMode.asBoolean

        val center: Translation2d = SingletonXboxController.getPOVasCorner()

        if (slowMode.asBoolean) {
            rot *= 0.6
        }

        swerveSubsystem.drive(Translation2d(0.0, 0.0), rot * controller.config.maxAngularVelocity, false, center)
    }

    /** @suppress */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    /** @suppress */
    override fun end(interrupted: Boolean) {}
}
