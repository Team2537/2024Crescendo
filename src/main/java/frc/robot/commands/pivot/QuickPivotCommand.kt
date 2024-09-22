package frc.robot.commands.pivot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.pivot.Pivot

/**
 * Command to pivot the arm to a specific angle
 * @param target the target angle to pivot to
 * @param auto does absolutely nothing
 * @param autoAim whether the angle should be calculated based on the limelight
 */
class QuickPivotCommand(
    private val pivotSubsystem: Pivot,
    private val target: Double,
    private val auto: Boolean,
    private val autoAim: Boolean
) : Command() {
    // These are used for an older version of the autoAim feature based on odometry
    /** @suppress */
    private var pose: Pose2d = Pose2d()
    /** @suppress */
    private var xDistanceMeters: Double = 0.0
    /** @suppress */
    var speakerPose: Pose2d = Constants.FIELD_LOCATIONS.SUBWOOFER_POSE
    /** @suppress */
    var targetAngle: Double = Constants.PivotConstants.INTAKE_POSITION

    private var direction: Boolean = false



    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(pivotSubsystem)
    }

    /**
     * Calculate the angle to pivot to based on the distance from the limelight
     * Determine the direction to pivot
     */
    override fun initialize() {
        // Set the target angle to the angle calculated based on the distance from the limelight if autoAim is true
        // Determine the direction to pivot
        direction = pivotSubsystem.getRelativePosition() > target
    }

    /**
     * Set the pivot to move up or down based on the direction
     */
    override fun execute() {
        if (pivotSubsystem.getRelativePosition() > target) {
            pivotSubsystem.pivotMotor.set(-0.2)
        } else {
            pivotSubsystem.pivotMotor.set(0.2)
        }
    }

    /**
     * Finish the command when the pivot reaches the target angle
     * Or moves past the target angle using the direction from the initialize method
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        if (direction) {
            return pivotSubsystem.getRelativePosition() < target
        } else {
            return pivotSubsystem.getRelativePosition() > target
        }
    }

    /**
     * Hold the pivot in place when the command is interrupted or canceled
     */
    override fun end(interrupted: Boolean) {
        pivotSubsystem.holdArm(target)
        println("Auto Pivot Stopped")
    }
}
