package frc.robot.util

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Constants


object SingletonXboxController : CommandXboxController(Constants.IOConstants.DRIVER_CONTROLLER_PORT){
    fun getPOVasCorner(): Translation2d {
        when(this.hid.pov){
            0 -> return Translation2d(Units.inchesToMeters(10.0), 0.0)
            45 -> return Translation2d(Units.inchesToMeters(10.0), -Units.inchesToMeters(10.0))
            90 -> return Translation2d(0.0, -Units.inchesToMeters(10.0))
            135 -> return Translation2d(-Units.inchesToMeters(10.0), -Units.inchesToMeters(10.0))
            180 -> return Translation2d(-Units.inchesToMeters(10.0), 0.0)
            225 -> return Translation2d(-Units.inchesToMeters(10.0), Units.inchesToMeters(10.0))
            270 -> return Translation2d(0.0, Units.inchesToMeters(10.0))
            315 -> return Translation2d(Units.inchesToMeters(10.0), Units.inchesToMeters(10.0))
        }
        return Translation2d(0.0, 0.0)
    }
}