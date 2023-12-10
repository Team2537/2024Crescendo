package frc.robot.subsystems

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase;

object LimelightSubsystem : SubsystemBase() {

    // NetworkTableEntry objects for getting data from the Limelight
    private var tx: NetworkTableEntry
    private var ty: NetworkTableEntry
    private var ta: NetworkTableEntry
    private var tv: NetworkTableEntry
    private var ts: NetworkTableEntry

    // NetworkTable object for getting data from the Limelight
    private var table: NetworkTable
    private var visionTab: ShuffleboardTab
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

        visionTab.addDouble("X Offset") { xOffset }
        visionTab.addDouble("Y Offset") { yOffset }
        visionTab.addDouble("Area") { area }
        visionTab.addDouble("Skew") { skew }
        visionTab.addBoolean("Target Visible") { targetVisible }

        // Create a Shuffleboard tab for the Limelight
        val visionTab: ShuffleboardTab = Shuffleboard.getTab("Vision")


    }

    override fun periodic() {
        // Update the data with the latest values from the Limelight
        xOffset = tx.getDouble(0.0)
        yOffset = ty.getDouble(0.0)
        area = ta.getDouble(0.0)
        skew = ts.getDouble(0.0)
        targetVisible = tv.getDouble(0.0) == 1.0
    }

    // Getters for the data from the Limelight
    fun getXOffset(): Double {
        return xOffset
    }

    fun getYOffset(): Double {
        return yOffset
    }

    fun getArea(): Double {
        return area
    }

    fun getSkew(): Double {
        return skew
    }

    fun isTargetVisible(): Boolean {
        return targetVisible
    }
}