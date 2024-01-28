package lib.zones

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import kotlinx.serialization.Serializable
import lib.serialization.Translation2dSerializer
import kotlin.math.max
import kotlin.math.min

/**
 * A rectangular bound that defines a specific zone with a purpose.
 *
 * Zones are stored as two points, and are strictly rectangular. The
 * tag of a zone denotes its purpose, while the display name (which
 * should be unique) exists to differentiate zones for the user.
 *
 * The coordinate system used by [this][Zone] class is the same as
 * [Translation2d], where x is "forward" and y is "sideways." More
 * specifically, the positive x-axis points away from the driver wall,
 * and the positive y-axis points perpendicularly left.
 *
 * Zones also support serialization with
 * [kotlinx.serialization](https://kotlinlang.org/docs/serialization.html)
 *
 * @author Matthew Clark
 *
 * @constructor Creates a zone with two given opposite corners and bounds
 * made from them.
 *
 * @see Translation2d
 * @see <a href="https://kotlinlang.org/docs/serialization.html">kotlinx</a>
 */
@Serializable
data class Zone(
    /**
     * The first corner of the rectangular bound of this zone.
     */
    @Serializable(with = Translation2dSerializer::class)
    val corner1: Translation2d,

    /**
     * The second corner of the rectangular bound of this zone.
     */
    @Serializable(with = Translation2dSerializer::class)
    val corner2: Translation2d,

    /**
     * A short tag to denote this zone's purpose.
     */
    val tag: String,

    /**
     * A unique display name to differentiate this zone from possible similar ones for
     * the user.
     */
    val displayName: String,
) {

    /**
     * Creates a zone with rectangular bounds defined by opposite corners at points
     * (x1, y1) and (x2, y2).
     *
     * @param x1 The first corner's x-coordinate
     * @param y1 The first corner's y-coordinate
     * @param x2 The second corner's x-coordinate
     * @param y2 The second corner's y-coordinate
     * @param tag A short tag to denote the zone's purpose
     * @param displayName A unique display name for the user
     */
    constructor(
        x1: Double, y1: Double,
        x2: Double, y2: Double,
        tag: String, displayName: String
    ) : this(Translation2d(x1, y1), Translation2d(x2, y2), tag, displayName)

    /**
     * Creates a zone with rectangular bounds defined by opposite corners at points
     * (x1, y1) and (x2, y2).
     *
     * @param x1 The first corner's x-coordinate
     * @param y1 The first corner's y-coordinate
     * @param x2 The second corner's x-coordinate
     * @param y2 The second corner's y-coordinate
     * @param tag A short tag to denote the zone's purpose
     * @param displayName A unique display name for the user
     */
    constructor(
        x1: Measure<Distance>, y1: Measure<Distance>,
        x2: Measure<Distance>, y2: Measure<Distance>,
        tag: String, displayName: String
    ) : this(Translation2d(x1, y1), Translation2d(x2, y2), tag, displayName)

    companion object {

    }

    /**
     * The position of the bottom left corner of this zone
     */
    val bottomLeft: Translation2d
        get() = Translation2d(bottomX, leftY)

    /**
     * The position of the top left corner of this zone
     */
    val topLeft: Translation2d
        get() = Translation2d(topX, leftY)

    /**
     * The position of the top right corner of this zone
     */
    val topRight: Translation2d
        get() = Translation2d(topX, rightY)

    /**
     * The position of the bottom right corner of this zone
     */
    val bottomRight: Translation2d
        get() = Translation2d(bottomX, rightY)

    /**
     * The x-coordinate of the bottom bound of this zone
     */
    val bottomX: Double
        get() = min(corner1.x, corner2.x)

    /**
     * The y-coordinate of the left bound of this zone
     */
    val leftY: Double
        get() = min(corner1.y, corner2.y)

    /**
     * The x-coordinate of the top bound of this zone
     */
    val topX: Double
        get() = max(corner1.x, corner2.x)

    /**
     * The y-coordinate of the right bound of this zone
     */
    val rightY: Double
        get() = max(corner1.y, corner2.y)

    /**
     * Checks if a [Translation2d] (a point) is contained within the
     * rectangular bounding provided by this zone.
     *
     * @param point The point on the field
     *
     * @return `true` if the point is within this zone, `false` if it
     * is outside it.
     */
    operator fun contains(point: Translation2d): Boolean {
        return (point.x in bottomX..topX) and
                (point.y in leftY..rightY)
    }

    /**
     * Checks if a [Pose2d] is contained within the rectangular
     * bounding provided by this zone
     *
     * @param pose The position on the field to check
     *
     * @return `true` if the point is within this zone, `false` if it
     * is outside it
     */
    operator fun contains(pose: Pose2d): Boolean {
        return pose.translation in this
    }

    /**
     * Checks if a [Pose3d] is contained within the rectangular
     * bounding provided by this zone
     *
     * @param pose The position on the field to check
     *
     * @return `true` if the point is within this zone, `false` if it
     * is outside it
     */
    operator fun contains(pose: Pose3d): Boolean {
        return pose.toPose2d() in this
    }

    /**
     * Checks if a point described by two given doubles is contained within the
     * rectangular bounding provided by this zone.
     *
     * @param x The x-coordinate of the point on the field
     * @param y The y-coordinate of the point on the field
     *
     * @return `true` if the point is within this zone, `false` if it
     * is outside it.
     */
    fun contains(x: Double, y: Double): Boolean {
        return (x in bottomX..topX) and
                (y in leftY..rightY)
    }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as Zone

        if (bottomLeft != other.bottomLeft) return false
        if (topRight != other.topRight) return false
        if (tag != other.tag) return false
        if (displayName != other.displayName) return false

        return true
    }

    override fun hashCode(): Int {
        var result = bottomLeft.hashCode()
        result = 31 * result + topRight.hashCode()
        return result
    }

    /**
     * Returns the display name of this zone. For its descriptive tag,
     * see [tag]
     *
     * @return The display name of this zone
     */
    override fun toString(): String {
        return displayName
    }
}
