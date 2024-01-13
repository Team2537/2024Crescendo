package frc.robot.commands.swerve

import frc.robot.subsystems.SwerveSubsystem
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.util.SingletonXboxController
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

class CornerSpinCommand(
    rotation: DoubleSupplier,
    driveMode: BooleanSupplier,
    slowMode: BooleanSupplier,
) : CommandBase() {
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

    override fun initialize() {}

    override fun execute() {
        var rot = rotation.asDouble
        val driveMode = driveMode.asBoolean

        val center: Translation2d = SingletonXboxController.getPOVasCorner()

        if (slowMode.asBoolean) {
            rot *= 0.6
        }

        swerveSubsystem.drive(Translation2d(0.0, 0.0), rot * controller.config.maxAngularVelocity, false, center)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
