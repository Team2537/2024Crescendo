package frc.robot.commands.vision

import SwerveSubsystem
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.util.SingletonXboxController
import kotlin.math.abs

class TrackTargetCommand : CommandBase() {
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

    override fun initialize() {}

    override fun execute() {
        rotation = pidController.calculate(limelightSubsystem.getXOffset(), 0.0)

        if ((abs(limelightSubsystem.getXOffset()) < 2 && limelightSubsystem.getArea() < 3.5) && limelightSubsystem.isTargetVisible()) {
            translation = Translation2d(0.3, -SingletonXboxController.leftX)
        } else {
            translation = Translation2d(0.0, -SingletonXboxController.leftX)
        }
        drivebase.drive(translation, rotation, false)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
