package lib.controllers

data class DriverProfile(
    val invertLeftX: Boolean,
    val invertLeftY: Boolean,
    val invertRightX: Boolean,
    val invertRightY: Boolean,
    val southpaw: Boolean,
    val leftPowerScale: Double,
    val rightPowerScale: Double,
)

fun defaultDriverProfile() =
    DriverProfile(
        invertLeftX = false,
        invertLeftY = false,
        invertRightX = false,
        invertRightY = false,
        southpaw = false,
        leftPowerScale = 1.0,
        rightPowerScale = 1.0,
    )
