package frc.robot.subsystems

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.vision.Limelight

object LimelightSubsystem : SubsystemBase() {
    private val limelight: Limelight = Limelight(NetworkTableInstance.getDefault().getTable("limelight"))


    // Variables for storing data from the Limelight
    private var xOffset: Double = 0.0
    private var yOffset: Double = 0.0
    private var area: Double = 0.0
    private var skew: Double = 0.0
    private var targetVisible: Boolean = false

    init {
        // Get the default NetworkTable for the Limelight
        table = NetworkTableInstance.getDefault().getTable("limelight")

        // Get the NetworkTableEntry objects for the Limelight
        tx = table.getEntry("tx")
        ty = table.getEntry("ty")
        ta = table.getEntry("ta")
        tv = table.getEntry("tv")
        ts = table.getEntry("ts")

        visionTab = Shuffleboard.getTab("Vision")

    val targetVisible: Boolean
        get() = limelight.targetVisible

}
