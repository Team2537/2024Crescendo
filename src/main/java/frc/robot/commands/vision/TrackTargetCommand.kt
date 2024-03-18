package frc.robot.commands.vision

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.util.SingletonXboxController
import kotlin.math.abs

/**
 * A command that will make the robot track a vision target from the limelight.
 *
 * @see SwerveSubsystem
 * @see LimelightSubsystem
 */
class TrackTargetCommand : Command() {
    private val limelightSubsystem = LimelightSubsystem
    private val drivebase = SwerveSubsystem
    private val pidController: PIDController

    private var rotation: Double = 0.0
    private var translation: Translation2d = Translation2d(0.0, 0.0)

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(limelightSubsystem, drivebase)
        pidController = PIDController(0.1, 0.0, 0.0)
    }

    /** @suppress */
    override fun initialize() {}

    /** @suppress */
    override fun execute() {
        rotation = pidController.calculate(limelightSubsystem.getTX(), 0.0)

        if ((abs(limelightSubsystem.getTX()) < 2 && limelightSubsystem.getTA() < 3.5) && limelightSubsystem.getTV()) {
            translation = Translation2d(0.3, -SingletonXboxController.leftX)
        } else {
            translation = Translation2d(0.0, -SingletonXboxController.leftX)
        }
        drivebase.drive(translation, rotation, false)
    }

    /** @suppress */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    /** @suppress */
    override fun end(interrupted: Boolean) {}
}
