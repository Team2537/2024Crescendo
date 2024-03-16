package frc.robot.commands.swerve

import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.SwerveSubsystem
import lib.getAngleTo
import lib.math.units.into
import kotlin.math.IEEErem
import kotlin.math.atan2

class AutoAlignCommand : Command() {
    private val swerveSubsystem = SwerveSubsystem
    private val pidController = PIDController(0.1, 0.001, 0.0)


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)
    }

    override fun initialize() {}

    override fun execute() {

        val currPose = swerveSubsystem.getPose()
        var relativePose = currPose.relativeTo(Constants.FIELD_LOCATIONS.SPEAKER_HOLE_POSE)
        if(DriverStation.getAlliance().isPresent){
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                relativePose = GeometryUtil.flipFieldPose(
                    currPose.relativeTo(Constants.FIELD_LOCATIONS.SPEAKER_HOLE_POSE))
            }
        }

        val angle = currPose.getAngleTo(relativePose)
        val speed = pidController.calculate(swerveSubsystem.getHeading().degrees, angle.into(Degrees))
        swerveSubsystem.drive(Translation2d(0.0, 0.0),
            speed * swerveSubsystem.getSwerveController().config.maxAngularVelocity,
            false)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return pidController.atSetpoint()
    }

    override fun end(interrupted: Boolean) {}
}
