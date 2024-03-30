/**
 * Copyright (c) 2024 Matthew Clark and FRC Robotics Team 2537 "Space Raiders"
 * All rights reserved
 *
 * - - -
 *
 * A simple extension set for the [edu.wpi.first.units] library.
 *
 * This extension set aims to reduce the boilerplate and verbosity of the
 * wpilib units library while maintaining its inherent type-safety and unit
 * conversion. The extension is built for the Kotlin programming language,
 * making heavy use of Kotlin's extension function syntax, which may not
 * work as intended in Java.
 *
 * A thank you to the other members of Team 2537 "Space Raiders" for helping avoid
 * naming conflicts while maintaining clarity.
 *
 * @author Matthew Clark
 */
package lib.math.units

import edu.wpi.first.math.geometry.*
import edu.wpi.first.units.Units.Rotation


/**
 * Gets the forward span of this translation
 *
 * @return the x coordinate as a [Span]
 * @see Translation3d.getX
 *
 * @since 2024-02-08
 */
val Translation3d.forward: Span
    get() = x.meters

/**
 * Gets the backwards span of this translation
 *
 * @return the -x coordinate as a [Span]
 * @see Translation3d.getX
 *
 * @since 2024-02-08
 */
val Translation3d.backwards: Span
    get() = -forward

/**
 * Gets the leftwards span of this translation
 *
 * @return the y coordinate as a [Span]
 * @see Translation3d.getY
 *
 * @since 2024-02-08
 */
val Translation3d.left: Span
    get() = y.meters

/**
 * Gets the rightwards span of this translation
 *
 * @return the -y coordinate as a [Span]
 * @see Translation3d.getY
 *
 * @since 2024-02-08
 */
val Translation3d.right: Span
    get() = -left

/**
 * Gets the upwards span of this translation
 *
 * @return the z coordinate as a [Span]
 * @see Translation3d.getZ
 *
 * @since 2024-02-08
 */
val Translation3d.up: Span
    get() = z.meters

/**
 * Gets the downwards span of this translation
 *
 * @return the -z coordinate as a [Span]
 * @see Translation3d.getZ
 *
 * @since 2024-02-08
 */
val Translation3d.down: Span
    get() = -up

/**
 * Gets the roll rotation of this translation
 *
 * @return the roll as a [Rotation]
 * @see Rotation3d.getX
 *
 * @since 2024-02-08
 */
val Rotation3d.roll: Rotation
    get() = x.radians

/**
 * Gets the pitch rotation of this translation
 *
 * @return the pitch as a [Rotation]
 * @see Rotation3d.getY
 *
 * @since 2024-02-08
 */
val Rotation3d.pitch: Rotation
    get() = y.radians

/**
 * Gets the yaw rotation of this translation
 *
 * @return the yaw as a [Rotation]
 * @see Rotation3d.getZ
 *
 * @since 2024-02-08
 */
val Rotation3d.yaw: Rotation
    get() = z.radians

/**
 * Gets the forward span of this translation
 *
 * @return the x coordinate as a [Span]
 * @see Translation2d.getX
 *
 * @since 2024-02-08
 */
val Translation2d.forward: Span
    get() = x.meters

/**
 * Gets the backwards span of this translation
 *
 * @return the -x coordinate as a [Span]
 * @see Translation2d.getX
 *
 * @since 2024-02-08
 */
val Translation2d.backwards: Span
    get() = -forward

/**
 * Gets the leftwards span of this translation
 *
 * @return the x coordinate as a [Span]
 * @see Translation2d.getX
 *
 * @since 2024-02-08
 */
val Translation2d.left: Span
    get() = y.meters

/**
 * Gets the rightward span of this translation
 *
 * @return the -y coordinate as a [Span]
 * @see Translation2d.getY
 *
 * @since 2024-02-08
 */
val Translation2d.right: Span
    get() = -left

/**
 * Gets the measure of this rotation
 *
 * @return the angle of this rotation as a [Rotation]
 *
 * @since 2024-02-08
 */
val Rotation2d.measure: Rotation
    get() = radians.radians

/**
 * Gets the forward span of this pose
 *
 * @return the x coordinate as a [Span]
 * @see Pose3d.getX
 * @see Translation3d.forward
 *
 * @since 2024-02-08
 */
val Pose3d.forward: Span
    get() = translation.forward

/**
 * Gets the backwards span of this pose
 *
 * @return the -x coordinate as a [Span]
 * @see Pose3d.getX
 * @see Translation3d.backwards
 *
 * @since 2024-02-08
 */
val Pose3d.backwards: Span
    get() = translation.backwards

/**
 * Gets the leftwards span of this pose
 *
 * @return the y coordinate as a [Span]
 * @see Pose3d.getY
 * @see Translation3d.left
 *
 * @since 2024-02-08
 */
val Pose3d.left: Span
    get() = translation.left

/**
 * Gets the rightward span of this pose
 *
 * @return the -y coordinate as a [Span]
 * @see Pose3d.getY
 * @see Translation3d.right
 *
 * @since 2024-02-08
 */
val Pose3d.right: Span
    get() = translation.right

/**
 * Gets the upward span of this pose
 *
 * @return the z coordinate as a [Span]
 * @see Pose3d.getZ
 * @see Translation3d.up
 *
 * @since 2024-02-08
 */
val Pose3d.up: Span
    get() = translation.up

/**
 * Gets the downward span of this pose
 *
 * @return the -z coordinate as a [Span]
 * @see Pose3d.getX
 * @see Translation3d.down
 *
 * @since 2024-02-08
 */
val Pose3d.down: Span
    get() = translation.down

/**
 * Gets the roll rotation of this pose
 *
 * @return the roll as a [Rotation]
 * @see Pose3d.getRotation
 * @see Rotation3d.roll
 *
 * @since 2024-02-08
 */
val Pose3d.roll: Rotation
    get() = rotation.roll

/**
 * Gets the pitch rotation of this pose
 *
 * @return the pitch as a [Rotation]
 * @see Pose3d.getRotation
 * @see Rotation3d.pitch
 *
 * @since 2024-02-08
 */
val Pose3d.pitch: Rotation
    get() = rotation.pitch

/**
 * Gets the yaw rotation of this pose
 *
 * @return the yaw as a [Rotation]
 * @see Pose3d.getRotation
 * @see Rotation3d.yaw
 *
 * @since 2024-02-08
 */
val Pose3d.yaw: Rotation
    get() = rotation.yaw

/**
 * Gets the forward span of this pose
 *
 * @return the x coordinate as a [Span]
 * @see Pose2d.getTranslation
 * @see Translation2d.forward
 *
 * @since 2024-02-08
 */
val Pose2d.forward: Span
    get() = translation.forward

/**
 * Gets the backwards span of this pose
 *
 * @return the -x coordinate as a [Span]
 * @see Pose2d.getTranslation
 * @see Translation2d.backwards
 *
 * @since 2024-02-08
 */
val Pose2d.backwards: Span
    get() = translation.backwards

/**
 * Gets the leftwards span of this pose
 *
 * @return the y coordinate as a [Span]
 * @see Pose2d.getTranslation
 * @see Translation2d.left
 *
 * @since 2024-02-08
 */
val Pose2d.left: Span
    get() = translation.left

/**
 * Gets the rightwards span of this pose
 *
 * @return the -y coordinate as a [Span]
 * @see Pose2d.getTranslation
 * @see Translation2d.right
 *
 * @since 2024-02-08
 */
val Pose2d.right: Span
    get() = translation.right

/**
 * Gets the angle of rotation of this pose
 *
 * @return the rotation as a [Span]
 * @see Pose2d.getRotation
 * @see Rotation2d.measure
 *
 * @since 2024-02-08
 */
val Pose2d.spin: Rotation
    get() = rotation.measure