package lib.pid

import com.revrobotics.SparkPIDController
import edu.wpi.first.math.controller.PIDController

fun PIDController.setGains(gains: PIDGains){
    this.p = gains.kP
    this.i = gains.kI
    this.d = gains.kD
}

fun SparkPIDController.setGains(gains: PIDGains){
    this.p = gains.kP
    this.i = gains.kI
    this.d = gains.kD
}

var PIDController.gains: PIDGains
    get() = PIDGains(p, i, d)
    set(value){ setGains(value) }

// Don't know why CANPIDController is deprecated while SparkPIDController, which implements it, is not,
// but whatever.
// Uncomment if we can/need to, but it doesn't seem to matter too much.
//var CANPIDController.gains: PIDGains
//    get() = PIDGains(p, i, d)
//    set(value) {
//        p = value.kP
//        i = value.kI
//        d = value.kD
//    }

var SparkPIDController.gains: PIDGains
    get() = PIDGains(p, i, d)
    set(value) { setGains(value) }
