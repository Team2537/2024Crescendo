package frc.robot.commands.swerve

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import lib.getAngleTo
import swervelib.SwerveController
import java.util.function.DoubleSupplier

class AbsoluteDriveElementRelativeCommand(
    target: Pose2d,
    vForward: DoubleSupplier,
    vStrafe: DoubleSupplier
) : Command() {
    private val swerveSubsystem = SwerveSubsystem
    private val target: Pose2d
    private val vForward: DoubleSupplier
    private val vStrafe: DoubleSupplier
    private val controller: SwerveController = swerveSubsystem.getSwerveController()
    private val anglePID = PIDController(0.1, 0.0, 0.0)


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
        this.target = target
        this.vForward = vForward
        this.vStrafe = vStrafe
    }

    override fun initialize() {}

    override fun execute() {
        val pose = swerveSubsystem.getPose()
        val angle = pose.getAngleTo(target)
        val rotation = anglePID.calculate(swerveSubsystem.getHeading().degrees, angle.`in`(Units.Degrees)) * controller.config.maxAngularVelocity

        val forwardVelocity = vForward.asDouble * swerveSubsystem.maximumSpeed
        val strafeVelocity = vStrafe.asDouble * swerveSubsystem.maximumSpeed




        swerveSubsystem.drive(Translation2d(forwardVelocity, strafeVelocity), rotation, true)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
