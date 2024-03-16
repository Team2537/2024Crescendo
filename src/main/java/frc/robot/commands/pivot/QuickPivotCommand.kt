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

class QuickPivotCommand(target: Double, auto: Boolean, autoAim: Boolean) : Command() {
    private val pivotSubsystem = PivotSubsystem
    private var target: Double
    private var direction: Boolean = false
    private val auto: Boolean
    private val autoAim: Boolean
    var speakerPose: Pose2d = Constants.FIELD_LOCATIONS.SUBWOOFER_POSE

    var targetAngle: Double = Constants.PivotConstants.INTAKE_POSITION



    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
        this.target = target
        this.auto = auto
        this.autoAim = autoAim
    }

    override fun initialize() {
        if(autoAim) {
            val pose = SwerveSubsystem.getPose()
            if (DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)) {
                speakerPose = GeometryUtil.flipFieldPose(Constants.FIELD_LOCATIONS.SUBWOOFER_POSE)
            } else {
                speakerPose = Constants.FIELD_LOCATIONS.SUBWOOFER_POSE
            }

            val relativePose = pose.relativeTo(speakerPose)
            val distanceMeters = Math.hypot(relativePose.translation.x, relativePose.translation.y)



            target = calculateAngle(distanceMeters.meters)

            println("Auto Aim Target: $target")
            println("Auto Aim Distance: $distanceMeters")
        }

        direction = pivotSubsystem.getRelativePosition() > target
    }

    override fun execute() {
        if(pivotSubsystem.getRelativePosition() > target){
            pivotSubsystem.pivotMotor.set(-0.2)
        } else {
            pivotSubsystem.pivotMotor.set(0.2)
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if(direction){
            return pivotSubsystem.getRelativePosition() < target
        } else {
            return pivotSubsystem.getRelativePosition() > target
        }
    }

    override fun end(interrupted: Boolean) {
        pivotSubsystem.holdArm(target)
        println("Auto Pivot Stopped")
    }
}
