package lib.zones

import edu.wpi.first.math.geometry.Translation2d
import kotlinx.serialization.Serializable
import kotlinx.serialization.SerializationStrategy
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
