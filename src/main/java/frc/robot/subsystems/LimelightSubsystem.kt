package frc.robot.subsystems

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.vision.Limelight

/**
 * The subsystem that controls the limelight.
 */
object LimelightSubsystem : SubsystemBase() {
    private val limelight: Limelight = Limelight(NetworkTableInstance.getDefault().getTable("limelight"))

    /**
     * Gets the horizontal offset of the target from the crosshair.
     */
    val xOffset: Double
        get() = limelight.xOffset

    /**
     * Gets the vertical offset of the target from the crosshair.
     */
    val yOffset: Double
        get() = limelight.yOffset

    /**
     * Gets the area of the camera's view that the target takes up.
     */
    val area: Double
        get() = limelight.area

    /** @suppress */
    val skew: Double
        get() = limelight.skew

    /**
     * Gets whether the limelight has a target in its view.
     */
    val targetVisible: Boolean
        get() = limelight.targetVisible
}