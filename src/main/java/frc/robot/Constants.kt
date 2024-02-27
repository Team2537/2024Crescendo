package frc.robot

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Filesystem
import lib.math.units.RotationVelocity
import lib.math.units.rpm
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

    object LauncherConstants {
        const val LEFT_LAUNCHER_PORT = 22
        const val RIGHT_LAUNCHER_PORT = 23

        const val ROLLER_MOTOR_PORT = 14
        const val ROLLER_P = 0.0001
        const val ROLLER_I = 0.000001

        const val INFRARED_SENSOR = 0

        val MINIMUM_VELOCITY: RotationVelocity = 6000.0.rpm
    }

    object PivotConstants {
        const val ABSOLUTE_ENCODER_PORT = 1
        const val PIVOT_MOTOR_PORT = 16

        const val ABSOLUTE_OFFSET = -0.052096177302404
        const val REL_ENCODER_CONVERSION = (40.0 / 18.0) * 36.0
        const val ABS_ENCODER_CONVERSION = (40.0 / 18.0)

        val distanceMap: HashMap<Double, Double> = hashMapOf(
            1.0 to 54.0
        )

        const val SUBWOOFER_POSITION = 0.276885539293289
        const val AMP_POSITION = 0.049505945295095
    }

    object IntakeConstants {
        const val INFRARED_SENSOR_CHANNEL = 1 //TODO: Update this

        const val INTAKE_MOTOR_ID: Int = 0 //TODO: Update this

        const val INTAKE_KP : Double = 0.1
        const val INTAKE_KI : Double = 0.0
        const val INTAKE_KD : Double = 0.0
    }
}
