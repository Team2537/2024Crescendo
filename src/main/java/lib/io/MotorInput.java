package lib.io;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import org.jetbrains.annotations.NotNull;

/**
 * Interface for getting input from a motor.
 * <p>
 * Implemented in java for reasons explained in {@link MotorOutput}
 */
public interface MotorInput {
    /**
     * Gets the current position of the motor.
     *
     * @return The position of the motor.
     */
    @NotNull
    Measure<Angle> getPosition();

    /**
     * Gets the current velocity of the motor.
     *
     * @return The velocity of the motor.
     */
    @NotNull
    Measure<Velocity<Angle>> getVelocity();
}