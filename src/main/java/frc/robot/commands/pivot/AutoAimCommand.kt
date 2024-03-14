package frc.robot.commands.pivot

import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.PivotSubsystem
import frc.robot.subsystems.SwerveSubsystem
import lib.calculateAngle
import lib.math.units.meters
import java.util.*

class AutoAimCommand(auto: Boolean) : Command() {
    private val pivotSubsystem = PivotSubsystem
    private var pose: Pose2d = Pose2d()
    private var xDistanceMeters: Double = 0.0
    private val auto: Boolean
    var speakerPose: Pose2d = Constants.FIELD_LOCATIONS.SUBWOOFER_POSE

    var targetAngle: Double = Constants.PivotConstants.INTAKE_POSITION


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
        this.auto = auto
    }

    override fun initialize() {
        pose = SwerveSubsystem.getPose()
        if(DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)) {
            speakerPose = GeometryUtil.flipFieldPose(speakerPose)
        }

        xDistanceMeters = Math.abs(speakerPose.x - pose.x)

        targetAngle = calculateAngle(xDistanceMeters.meters)
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true
    }

    override fun end(interrupted: Boolean) {}
}
