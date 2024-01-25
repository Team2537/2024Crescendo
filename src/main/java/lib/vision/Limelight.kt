package lib.vision

import edu.wpi.first.networktables.DoubleSubscriber
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
class Limelight(table: NetworkTable) : AutoCloseable {
    // NetworkTableEntry objects for getting data from the Limelight
    private val tx: DoubleSubscriber
    private val ty: DoubleSubscriber
    private val ta: DoubleSubscriber
    private val ts: DoubleSubscriber
    private val tv: DoubleSubscriber

    private val visionTab: ShuffleboardTab

    init {
        // Get the NetworkTableEntry objects for the Limelight
        tx = table.getDoubleTopic("tx").subscribe(0.0)
        ty = table.getDoubleTopic("ty").subscribe(0.0)
        ta = table.getDoubleTopic("ta").subscribe(0.0)
        ts = table.getDoubleTopic("ts").subscribe(0.0)
        tv = table.getDoubleTopic("tv").subscribe(0.0)

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
        get() = tx.get()

    /**
     * Gets the offset of the cross-hair to the target on the y-axis.
     * Up is positive
     *
     * @return the y offset
     */
    val yOffset: Double
        get() = ty.get()

    // TODO 0.0-1.0 or 0.0-100.0 ??
    /**
     * Gets the percentage of the camera field of view that the target is visible
     *
     * @return how much of the screen can see the target
     */
    val area: Double
        get() = ta.get()

    /**
     * Returns the skew of the bounding box from 0 to 90 degrees, essentially how 'crooked'
     * it is.
     *
     * @return the skew/roll of the bounding box from 0 to 90 degrees
     */
    val skew: Double
        get() = ts.get()

    /**
     * Checks whether the target object is visible to the limelight camera
     *
     * @return `true` if the target object is visible to the limelight camera,
     * `false` otherwise.
     */
    val targetVisible: Boolean
        get() = tv.get() == 1.0

    override fun close() {
        // Not entirely necessary, as most limelights will have the same lifespan as the robot, but still
        // worth considering before someone forgets that this is even a thing you have to do.
        tx.close()
        ty.close()
        ta.close()
        tv.close()
    }
}
