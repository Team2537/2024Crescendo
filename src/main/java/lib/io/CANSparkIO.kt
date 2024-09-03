package lib.io

import com.revrobotics.CANSparkBase
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Rotations
import lib.math.units.*

/**
 * Interface for interacting with CANSpark motors specifically.
 */
class CANSparkIO(private val motor: CANSparkBase) : MotorIO {
    private val encoder = motor.encoder
    private val pid = motor.pidController

    override var position: Rotation
        get() = encoder.position.rotations
        set(value) {
            // Use PID to attempt getting to position.
            pid.setReference(value into Rotations, CANSparkBase.ControlType.kPosition)
        }

    override var velocity: RotationVelocity
        get() = encoder.velocity.rpm
        set(value) {
            pid.setReference(value into RPM, CANSparkBase.ControlType.kVelocity)
        }

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false

        other as CANSparkIO

        if (motor != other.motor) return false

        return true
    }

    override fun hashCode(): Int {
        return motor.hashCode()
    }

    override fun toString(): String {
        return "CANSparkIO(position=$position, velocity=$velocity)"
    }
}