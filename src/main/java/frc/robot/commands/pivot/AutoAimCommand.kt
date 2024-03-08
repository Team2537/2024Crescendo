package frc.robot.commands.pivot

import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.PivotSubsystem
import frc.robot.subsystems.SwerveSubsystem
import java.sql.Driver
import java.util.*

class AutoAimCommand : Command() {
    private val pivotSubsystem = PivotSubsystem
    private var pose: Pose2d = Pose2d()
    private var xDistanceMeters: Double = 0.0
    var speakerPose: Pose2d = Constants.FIELD_LOCATIONS.SUBWOOFER_POSE


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
    }

    override fun initialize() {
        pose = SwerveSubsystem.getPose()
        if(DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)) {
            speakerPose = GeometryUtil.flipFieldPose(speakerPose)
        }

        xDistanceMeters = Math.abs(speakerPose.x - pose.x)

        println(PivotSubsystem.getInterpolatedAngle(xDistanceMeters))


        println("Absolute Value of ${speakerPose.x} - ${pose.x} = $xDistanceMeters")
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true
    }

    override fun end(interrupted: Boolean) {}
}
