package frc.robot

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Filesystem
import swervelib.math.Matter
import swervelib.parser.PIDFConfig
import java.io.File

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

object Constants {
    const val ROBOT_MASS = (148 - 20.3) * 0.453592 // 32lbs * kg per pound
    val CHASSIS = Matter(Translation3d(0.0, 0.0, Units.inchesToMeters(8.0)), ROBOT_MASS)
    const val LOOP_TIME = 0.13 // s, 20ms + 110ms spark max velocity lag

    object OperatorConstants {
        const val LEFT_X_DEADBAND = 0.01
        const val LEFT_Y_DEADBAND = 0.01
    }

    object Auto {
        val xAutoPID: PIDFConfig = PIDFConfig(0.7, 0.0, 0.0)
        val yAutoPID: PIDFConfig = PIDFConfig(0.7, 0.0, 0.0)
        val angleAutoPID: PIDFConfig = PIDFConfig(0.4, 0.0, 0.01)
        const val MAX_SPEED = 4.0
        const val MAX_ACCELERATION = 2.0
    }

    object Drivebase {
        // Hold time on motor brakes when disabled
        const val WHEEL_LOCK_TIME = 10.0 // seconds
    }

    object IOConstants {
        const val DRIVER_CONTROLLER_PORT = 0
        const val OPERATOR_CONTROLLER_PORT = 1
    }

    object FileConstants {
        val BOUNTY_CONFIG: File = File(Filesystem.getDeployDirectory(), "/swerve/bounty")
    }

    object LauncherConstants{
        const val ANGLE_MOTOR_PORT = 3
        const val ANGLE_KP = 0.001
        const val ANGLE_KI = 0.0
        const val ANGLE_KD = 0.0

        const val LEFT_LAUNCHER_PORT : Int = 1
        const val RIGHT_LAUNCHER_PORT : Int = 2
        const val LAUNCHER_P : Double = 0.0002
        const val LAUNCHER_I : Double = 0.00001
        const val LAUNCHER_D : Double = 0.00001

        const val STORE_MOTOR_PORT : Int = 4
        const val STORE_P = 0.0002
        const val STORE_I = 0.00001
        const val STORE_D = 0.00001

        const val ABSOLUTE_ENCODER_PORT = 0


    }
}
