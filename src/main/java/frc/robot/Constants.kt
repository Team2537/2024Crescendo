package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
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
    val ROBOT_MASS = Units.lbsToKilograms(148.0)
    val CHASSIS = Matter(Translation3d(0.0, 0.0, Units.inchesToMeters(8.0)), ROBOT_MASS)
    const val LOOP_TIME = 0.13 // s, 20ms + 110ms spark max velocity lag

    const val GEARBOX_RATIO = 36.0
    const val PULLEY_RATIO = 40.0/18.0

    object OperatorConstants {
        const val LEFT_X_DEADBAND = 0.01
        const val LEFT_Y_DEADBAND = 0.01

        const val BACK_BUTTON = 7
        const val START_BUTTON = 8
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
        val CRESCENDO_CONFIG: File = File(Filesystem.getDeployDirectory(), "/swerve/crescendo")
    }

    object LauncherConstants {
        const val TOP_FLYWHEELS = 23
        const val BOTTOM_FLYWHEELS = 22
        const val ROLLER_MOTOR = 14

        const val LEFT_NOTE_DETECTOR = 0
        const val RIGHT_NOTE_DETECTOR = 3
    }

    object PivotConstants {

        const val GEARBOX_RATIO: Double = 36.0/1.0
        const val PULLEY_RATIO: Double = 40.0/18.0

        const val ABSOLUTE_ENCODER_PORT = 1
        const val PIVOT_MOTOR_PORT = 16
        const val HOMING_SENSOR_PORT = 2



        const val ABSOLUTE_OFFSET = 0.320913258022831
        const val REL_ENCODER_CONVERSION = 360 / (GEARBOX_RATIO * PULLEY_RATIO)
        const val ABS_ENCODER_CONVERSION = 360 / PULLEY_RATIO

        const val kG = 0.39
        const val kV = 1.56
        const val kA = 0.02
        const val kS = 0.1

        const val kP = 0.5
        const val kI = 0.01
        const val kD = 0.0

        const val SUBWOOFER_POSITION = 79.5
        const val AMP_POSITION = 0.5
        const val INTAKE_POSITION = 71.5
        const val PODIUM_POSITION = 57.0 // Ish
        const val MID_POSITION = 63.2

        const val slope = -0.33

        val distanceMap: HashMap<Double, Double> = hashMapOf(
            Pair(0.0 + 48.5, 81.5),
            Pair(25.5 + 48.5, 69.1),
            Pair(69.0 + 48.5, 61.7),
            Pair(94.0 + 48.5, 57.0),
        )
    }

    object IntakeConstants {
        const val INTAKE_MOTOR_ID: Int = 20 //TODO: Update this
        const val TRANSFER_MOTOR_ID: Int = 17 //TODO: Update this
    }

    object ClimbConstants {
        const val MOTOR_SPEED_UP = 0.3
        const val MOTOR_SPEED_DOWN = -0.3
        const val LEFT_CLIMB_PORT = 18 // placeholder value, replace with actual port
        const val RIGHT_CLIMB_PORT = 15 // placeholder value, replace with actual port
        const val ARMS_UP_ENCODER_POSITION = 0.0
        const val ARMS_DOWN_ENCODER_POSITION = 0.0
        const val ARMS_ENCODER_TOLERANCE = 0.1
        const val START_POSITION = 7
    }

    object FIELD_LOCATIONS {
        val SUBWOOFER_POSE = Pose2d(1.38, 5.55, Rotation2d.fromDegrees(0.0))
    }
}
