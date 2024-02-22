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