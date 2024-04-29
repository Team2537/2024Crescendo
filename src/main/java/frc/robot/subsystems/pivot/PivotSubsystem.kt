package frc.robot.subsystems.pivot

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier

object PivotSubsystem : SubsystemBase() {
    val io: PivotIO
    val inputs: PivotIO.PivotIOInputs

    val mech = Mechanism2d(3.0, 3.0)
    val root = mech.getRoot("pivot_arm", 2.0, 0.0)
    val arm_base = root.append(MechanismLigament2d("arm_base", 1.0, 90.0))
    val wrist = arm_base.append(MechanismLigament2d("wrist", 0.5, 0.0))

    init {
        if (RobotBase.isReal()) {
            io = PivotIONeos()
        } else {
            io = PivotIOSim() // Fix this later
        }

        inputs = PivotIO.PivotIOInputs()
    }

    fun homingRoutine(): Command? {
        return this.runEnd(
            { io.setRawVoltage(Units.Volts.of(-0.2 * 12)) },
            { io.stop() }
        ).until { inputs.homingSensorTriggered }
    }

    fun manualPivot(speed: DoubleSupplier): Command? {
        return this.runEnd(
            { io.setRawVoltage(Units.Volts.of(speed.asDouble * 6.0)) },
            { io.stop() }
        )
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Pivot", inputs)
        wrist.angle = inputs.relativeAngle.into(Units.Degrees)
        Logger.recordOutput("Pivot/Mechanism", mech)
    }
}