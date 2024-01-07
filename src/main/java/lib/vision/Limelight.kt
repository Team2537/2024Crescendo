package lib.vision

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab

/**
 * Represents a single limelight unit (vision).
 *
 * The limelight gets its data from a [NetworkTable].
 * [This class][Limelight] allows simple access to
 * various tables through more descriptive name and
 * properties rather than raw [NetworkTable.getEntry].
 * It also manages its own [ShuffleboardTab] to
 * display information about the limelight unit.
 *
 * @author Matthew Clark
 */
class Limelight(table: NetworkTable) {
    // NetworkTableEntry objects for getting data from the Limelight
    private var tx: NetworkTableEntry
    private var ty: NetworkTableEntry
    private var ta: NetworkTableEntry
    private var tv: NetworkTableEntry
    private var ts: NetworkTableEntry

    private var visionTab: ShuffleboardTab

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

    /**
     * Gets the offset of the cross-hair to the target on the x-axis.
     * Right is positive
     *
     * @return the x offset
     */
    val xOffset: Double
        get() = tx.getDouble(0.0)

    /**
     * Gets the offset of the cross-hair to the target on the y-axis.
     * Up is positive
     *
     * @return the y offset
     */
    val yOffset: Double
        get() = ty.getDouble(0.0)

    // TODO 0.0-1.0 or 0.0-100.0 ??
    /**
     * Gets the percentage of the camera field of view that the target is visible
     *
     * @return how much of the screen can see the target
     */
    val area: Double
        get() = ta.getDouble(0.0)

    // FIXME: Potentially deprecated; what did it even do?
    @Deprecated("Lack of documentation")
    val skew: Double
        get() = ts.getDouble(0.0)

    /**
     * Checks whether the target object is visible to the limelight camera
     *
     * @return `true` if the target object is visible to the limelight camera,
     * `false` otherwise.
     */
    val targetVisible: Boolean
        get() = tv.getDouble(0.0) == 1.0
}
