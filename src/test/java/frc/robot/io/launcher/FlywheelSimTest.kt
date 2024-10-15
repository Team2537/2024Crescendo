package frc.robot.io.launcher

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.RPM
import frc.robot.subsystems.launcher.flywheels.FlywheelIO
import frc.robot.subsystems.launcher.flywheels.FlywheelIOSim
import lib.ControllerGains
import lib.math.units.into
import org.junit.jupiter.api.Test

internal class FlywheelSimTest {
    @Test
    fun speedsUp() {
        val io = FlywheelIOSim(
            DCMotor.getNeoVortex(1),
            0.001317 * 4,
            ControllerGains()
        )

        val inputs = FlywheelIO.FlywheelInputs()

        io.setVoltage(Units.Volts.of(12.0))
        for (i in 0..200) {
            io.updateInputs(inputs)
        }

        println(inputs.velocity into RPM)
        assert((inputs.velocity into RPM) > 6000)
    }

    @Test
    fun velocityControl() {
        val io = FlywheelIOSim(
            DCMotor.getNeoVortex(1),
            0.001317 * 4,
            ControllerGains(
                kV = 12.0/6900.0,
                kP = 0.17
            )
        )

        val inputs = FlywheelIO.FlywheelInputs()

        io.setVelocity(Units.RPM.of(6000.0))
        for (i in 0..2000) {
            io.updateInputs(inputs)
        }

        println(inputs.velocity into RPM)
        assert(inputs.velocity.isNear(Units.RPM.of(6000.0), 0.1))
    }
}