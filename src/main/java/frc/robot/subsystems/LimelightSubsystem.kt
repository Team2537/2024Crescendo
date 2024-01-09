package frc.robot.subsystems

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.vision.Limelight

object LimelightSubsystem : SubsystemBase() {
    private val limelight: Limelight = Limelight(NetworkTableInstance.getDefault().getTable("limelight"))

    val xOffset: Double
        get() = limelight.xOffset

    val yOffset: Double
        get() = limelight.yOffset

    val area: Double
        get() = limelight.area

    val skew: Double
        get() = limelight.skew

    val targetVisible: Boolean
        get() = limelight.targetVisible
}