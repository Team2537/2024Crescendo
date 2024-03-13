package lib.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.networktables.DoubleArraySubscriber
import edu.wpi.first.networktables.DoubleSubscriber
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.Degrees
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
class Limelight(hostname: String, mountPosition: Pose3d) : AutoCloseable {
    // NetworkTableEntry objects for getting data from the Limelight
    private val tx: DoubleSubscriber
    private val ty: DoubleSubscriber
    private val ta: DoubleSubscriber
    private val ts: DoubleSubscriber
    private val tv: DoubleSubscriber

    private val table: NetworkTable

    private val botpose: DoubleArraySubscriber

    private val visionTab: ShuffleboardTab

    private var _mountPosition: Pose3d
    var mountPosition: Pose3d
        get() = _mountPosition
        set(value) {
            _mountPosition = value
        }

    init {
        table = NetworkTableInstance.getDefault().getTable(hostname)
        // Get the NetworkTableEntry objects for the Limelight
        tx = table.getDoubleTopic("tx").subscribe(0.0)
        ty = table.getDoubleTopic("ty").subscribe(0.0)
        ta = table.getDoubleTopic("ta").subscribe(0.0)
        ts = table.getDoubleTopic("ts").subscribe(0.0)
        tv = table.getDoubleTopic("tv").subscribe(0.0)

        botpose = table.getDoubleArrayTopic("botpose").subscribe(DoubleArray(6))

        visionTab = Shuffleboard.getTab("Vision")

        visionTab.addDouble("X Offset") { yawRaw }
        visionTab.addDouble("Y Offset") { pitchRaw }
        visionTab.add("Yaw Offset") { yawOffset }
        visionTab.add("Pitch Offset") { pitchOffset }
        visionTab.add("Roll") { roll }
        visionTab.add("Position") { position }
        visionTab.addDouble("Area") { area }
        visionTab.addBoolean("Target Visible") { targetVisible }

        _mountPosition = mountPosition
        // FIXME - remove if possible
        // Create a Shuffleboard tab for the Limelight
        val visionTab: ShuffleboardTab = Shuffleboard.getTab("Vision")
    }

    /**
     * nothing for now
     *
     * *Will* configure mounting position from the [frc.robot.Constants] object
     */
    fun configureMountPosition(){
        //TODO: When things get mounted, do this
//        _mountPosition = Pose3d()
    }

    /**
     * Gets the current position of the limelight estimated by
     * the april tags.
     *
     * @return the estimated position of the limelight
     * @see Pose3d
     */
     val position: Pose3d
        get() {
            val results: DoubleArray = botpose.get()

            val pos = Translation3d(results[0], results[1], results[2])

            return Pose3d(
                pos,
                Rotation3d(
                    results[3],
                    results[4],
                    results[5],
                )
            )
        }

    val poseMeasurement: VisionMeasurement
        get() {
            val results = botpose.get()

            val pos = Translation3d(results[0], results[1], results[2])

            val pose = Pose3d(
                pos,
                Rotation3d(
                    results[3],
                    results[4],
                    results[5],
                )
            )

            return VisionMeasurement(pose, latency = results[6])
        }

    /**
     * Gets the offset of the cross-hair to the target on the x-axis.
     * Right is positive
     *
     * @return the x offset
     */
    private val yawRaw: Double
        get() = tx.get()

    /**
     * Gets the angle
     */
    val yawOffset: Measure<Angle>
        get() = Degrees.of(yawRaw)

    /**
     * Gets the offset of the cross-hair to the target on the y-axis.
     * Up is positive
     *
     * @return the y offset
     */
    private val pitchRaw: Double
        get() = ty.get()

    val pitchOffset: Measure<Angle>
        get() = Degrees.of(pitchRaw)

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
    private val skewRaw: Double
        get() = ts.get()

    val roll: Measure<Angle>
        get() = Degrees.of(skewRaw)

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
        ts.close()
        tv.close()
    }
}
