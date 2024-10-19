package frc.robot.io.launcher

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import frc.robot.subsystems.superstructure.launcher.roller.RollerIO
import frc.robot.subsystems.superstructure.launcher.roller.RollerIOSim
import lib.ControllerGains
import lib.math.units.into
import org.junit.jupiter.api.Test

internal class RollerSimTest {
    @Test
    fun setRollerVoltage(){
        val io = RollerIOSim(
            motor = DCMotor.getNEO(1),
            moiKgM2 = 0.0000434119,
            rollerRadius = Inches.of(1.460),
            gains = ControllerGains()
        )

        val inputs: RollerIO.RollerInputs = RollerIO.RollerInputs()

        io.setVoltage(Volts.of(12.0))

        for (i in 0..200){
            io.updateInputs(inputs)
        }

        println("Velocity: ${inputs.velocity into RPM}")
        assert(inputs.velocity > RotationsPerSecond.of(0.0))
    }

    fun setRollerPosition(){
        val io = RollerIOSim(
            motor = DCMotor.getNEO(1),
            moiKgM2 = 0.0000434119,
            rollerRadius = Inches.of(1.460),
            gains = ControllerGains(kP = 0.02, kD = 5.0)
        )

        val inputs: RollerIO.RollerInputs = RollerIO.RollerInputs()

        io.setTargetPosition(Inches.of(6.0))

        for (i in 0..200){
            io.updateInputs(inputs)
        }

        println("Position: ${inputs.position into Degrees}")
        assert(inputs.position > Rotations.of(0.0))
    }
}