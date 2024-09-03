package lib.io

import lib.math.units.Rotation
import lib.math.units.RotationVelocity

/**
 * A kotlin interface for simpler implementation when creating motor ios
 * outside of java. This interface simply wraps the separated get and set
 * methods with default behaviour into identical properties.
 */
interface MotorIO : MotorInput, MotorOutput {

    /**
     * The position of the motor. Controlling this will cause the motor to attempt movement.
     */
    var position: Rotation

    /**
     * The velocity of the motor. Controlling this will cause the motor to attempt movement.
     */
    var velocity: RotationVelocity

    override fun getPosition(): Rotation = position

    override fun setPosition(position: Rotation) {
        this.position = position
    }

    override fun getVelocity(): RotationVelocity = velocity

    override fun setVelocity(velocity: RotationVelocity) {
        this.velocity = velocity
    }
}