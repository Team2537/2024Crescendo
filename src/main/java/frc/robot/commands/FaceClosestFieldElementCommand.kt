package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import lib.getAngleTo

class FaceClosestFieldElementCommand(elements: List<Pose2d>) : Command() {
    private val swerveSubsystem = SwerveSubsystem
    private val elements: List<Pose2d>

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
        this.elements = elements
    }

    override fun initialize() {}

    override fun execute() {
        val pose = swerveSubsystem.getPose()
        val closestElement = pose.nearest(elements)
        val angle = pose.getAngleTo(closestElement).`in`(Degrees)
        println(angle)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
