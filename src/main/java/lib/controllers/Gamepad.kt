package lib.controllers

interface Gamepad {
    val leftX: Double
    val leftY: Double
    val rightX: Double
    val rightY: Double
    val leftTrigger: Double
    val rightTrigger: Double
}