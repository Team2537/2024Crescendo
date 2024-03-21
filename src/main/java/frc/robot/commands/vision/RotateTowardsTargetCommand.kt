package frc.robot.commands.vision

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.subsystems.SwerveSubsystem
import lib.math.units.into
import lib.vision.Limelight

/**
 * A command that will make the robot track a vision target from the limelight.
 *
 * @see SwerveSubsystem
 * @see LimelightSubsystem
 */
class RotateTowardsTargetCommand(private val limelight: Limelight) : Command() {
    private val limelightSubsystem = LimelightSubsystem
    private val drivebase = SwerveSubsystem
    private val pidController: PIDController

    private var rotation: Double = 0.0
    private var target = 0.0

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(limelightSubsystem, drivebase)
        pidController = PIDController(0.1, 1e-8, 0.0)
    }

    /** @suppress */
    override fun initialize() {
        target = SwerveSubsystem.getHeading().degrees - (limelight.absoluteTX into Degrees)
    }

    /** @suppress */
    override fun execute() {
        rotation = pidController.calculate(SwerveSubsystem.getHeading().degrees, target)

        drivebase.drive(Translation2d(), rotation, false)
    }

    /** @suppress */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return pidController.atSetpoint()
    }

    /** @suppress */
    override fun end(interrupted: Boolean) {}
}
