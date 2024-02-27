package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.vision.Limelight

/**
 * The subsystem that controls the limelight.
 */
object LimelightSubsystem : SubsystemBase() {
    // TODO: get the actual position of the limelight
    private val limelight: Limelight = Limelight(NetworkTableInstance.getDefault().getTable("limelight"), Pose3d())


    /**
     * Get the position of the bot as estimated by the limelight
     */
    val botpose: Pose3d
        get() = limelight.position
    

    /**
     * Gets the horizontal offset of the target from the crosshair.
     */
    val xOffset: Measure<Angle>
        get() = limelight.yawOffset

    /**
     * Gets the vertical offset of the target from the crosshair.
     */
    val yOffset: Measure<Angle>
        get() = limelight.pitchOffset

    /**
     * Gets the area of the camera's view that the target takes up.
     */
    val area: Double
        get() = limelight.area

    /** @suppress */
    val skew: Measure<Angle>
        get() = limelight.roll

    /**
     * Gets whether the limelight has a target in its view.
     */
    val targetVisible: Boolean
        get() = limelight.targetVisible
}
