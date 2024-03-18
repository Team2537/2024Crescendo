package frc.robot.commands.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.subsystems.SwerveSubsystem
import lib.vision.VisionMeasurement

/**
 * A command to update the odometry with the latest vision
 * measurements from the [LimelightSubsystem]
 *
 * @author Matthew Clark
 *
 * @see LimelightSubsystem
 */
class UpdateOdometryCommand : Command() {

    init {
        addRequirements(LimelightSubsystem)
    }

    override fun execute() {
        val estimate = LimelightSubsystem.getPoseEsimate()
        if(estimate.tagCount >= 2){
            SwerveSubsystem.setVisionMeasurementStdDevs(.7, .7, 99999999.0)
            SwerveSubsystem.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds - estimate.latency
            )
        }

    }
    
    override fun isFinished(): Boolean {
        return false
    }
}