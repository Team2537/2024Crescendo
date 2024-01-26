package lib.zones

import edu.wpi.first.math.geometry.Translation2d
import kotlinx.serialization.Serializable
import lib.serialization.Translation2dSerializer

/**
 * A rectangular bound that defines a specific zone with a purpose.
 *
 * Zones are stored as two points, and are strictly rectangular. The
 * tag of a zone denotes its purpose, while the display name (which
 * should be unique) exists to differentiate zones for the user.
 *
 * @author Matthew Clark
 */
@Serializable
data class Zone(
    /**
     * The bottom left corner of the rectangular bound of this zone.
     */
    @Serializable(with = Translation2dSerializer::class)
    val bottomLeft: Translation2d,

    /**
     * The top right corner of the rectangular bound of this zone.
     */
    @Serializable(with = Translation2dSerializer::class)
    val topRight: Translation2d,

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
     * Checks if a [Translation2d] (a point) is contained within the
     * rectangular bounding provided by this zone.
     *
     * @param point The point on the field
     *
     * @return `true` if the point is within this zone, `false` if it
     * is outside of it.
     */
    operator fun contains(point: Translation2d): Boolean {
        return (point.x in bottomLeft.x..topRight.x) and
                (point.y in bottomLeft.y..topRight.y)
    }

    /**
     * Checks if a point described by two given doubles is contained within the
     * rectangular bounding provided by this zone.
     *
     * @param x The x-coordinate of the point on the field
     * @param y The y-coordinate of the point on the field
     *
     * @return `true` if the point is within this zone, `false` if it
     * is outside of it.
     */
    fun contains(x: Double, y: Double): Boolean {
        return (x in bottomLeft.x..topRight.x) and
                (y in bottomLeft.y..topRight.y)
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
