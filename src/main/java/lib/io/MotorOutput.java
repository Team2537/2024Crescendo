package lib.io;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import org.jetbrains.annotations.NotNull;

/**
 * Interface for outputting to a motor.
 * <p>
 * Implemented in java to avoid conflict with kotlin integration between
 * methods and properties; it would be impossible to isolate just the set
 * methods with kotlin properties.
 */
public interface MotorOutput {

    /**
     * Sets a desired target position for the motor to reach and maintain.
     *
     * @param position The desired position.
     */
    void setPosition(@NotNull Measure<Angle> position);

    /**
     * Sets a desired velocity to reach and maintain.
     *
     * @param velocity The desired velocity.
     */
    void setVelocity(@NotNull Measure<Velocity<Angle>> velocity);
}
