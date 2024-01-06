package lib

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab

class Limelight(table: NetworkTable) {
    // NetworkTableEntry objects for getting data from the Limelight
    private var tx: NetworkTableEntry
    private var ty: NetworkTableEntry
    private var ta: NetworkTableEntry
    private var tv: NetworkTableEntry
    private var ts: NetworkTableEntry

    private var visionTab: ShuffleboardTab

    // Variables for storing data from the Limelight
    private var _xOffset: Double = 0.0
    private var _yOffset: Double = 0.0

    private var _area: Double = 0.0
    private var _skew: Double = 0.0
    private var _targetVisible: Boolean = false

    init {
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

        // FIXME - remove if possible
        // Create a Shuffleboard tab for the Limelight
        val visionTab: ShuffleboardTab = Shuffleboard.getTab("Vision")
    }

    val xOffset: Double
        get() = tx.getDouble(0.0)
    val yOffset: Double
        get() = ty.getDouble(0.0)

    val area: Double
        get() = ta.getDouble(0.0)

    val skew: Double
        get() = ts.getDouble(0.0)

    val targetVisible: Boolean
        get() = tv.getDouble(0.0) == 1.0
}
