package frc.robot.subsystems.climb

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier

object ClimberSubsystem : SubsystemBase() {
    val io: ClimbIO
    val input: ClimbIO.ClimbIOInputs = ClimbIO.ClimbIOInputs()

    val arms: Mechanism2d = Mechanism2d(6.0, 6.0)
    val leftArmRoot: MechanismRoot2d = arms.getRoot("left", 2.0, 0.0)
    val rightArmRoot: MechanismRoot2d = arms.getRoot("right", 4.0, 0.0)

    val leftArm = leftArmRoot.append(
        MechanismLigament2d(
            "left_arm", 1.0, 90.0
        )
    )
    val rightArm = rightArmRoot.append(
        MechanismLigament2d(
            "right_arm", 1.0, 90.0
        )
    )

    val tab = Shuffleboard.getTab("Climber")

    init {
        if (RobotBase.isReal()) {
            io = ClimbIONeos()
        } else {
            // Put a simulation here
            io = ClimbIOSim()
        }
        tab.add("Arms", arms)
    }

    fun individualArmControl(leftPower: DoubleSupplier, rightPower: DoubleSupplier): Command? {
        return this.runEnd({
            io.setLeftArmPower(leftPower.asDouble)
            io.setRightArmPower(rightPower.asDouble)
        }, {
            io.stop()
        })
    }

    fun dualArmControl(power: DoubleSupplier): Command? {
        return this.runEnd({
            io.setLeftArmPower(power.asDouble)
            io.setRightArmPower(power.asDouble)
        }, {
            io.stop()
        })
    }

    fun clearEncoders(): Command? {
        return this.runOnce {
            io.resetLeftArmPosition()
            io.resetRightArmPosition()
        }
    }

    override fun periodic() {
        io.updateInputs(input)
        leftArm.length = input.leftArmPosition.into(Units.Inches)
        rightArm.length = input.rightArmPosition.into(Units.Inches)
        Logger.processInputs("Climber", input)
    }
}