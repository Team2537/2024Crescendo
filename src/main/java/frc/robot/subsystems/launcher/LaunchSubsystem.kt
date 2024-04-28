package frc.robot.subsystems.launcher

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

object LaunchSubsystem : SubsystemBase() {
    val io: LauncherIO
    val input: LauncherIO.LauncherIOInputs = LauncherIO.LauncherIOInputs()

    /** Feedforward controller for the top flywheel */
    val topFlywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(-0.20832, 0.11109, 0.024896)

    /** Feedforward controller for the bottom flywheel */
    val bottomFlywheelFeedforward: SimpleMotorFeedforward = SimpleMotorFeedforward(0.035079, 0.10631, 0.0080339)

    init {
        io = LauncherIONeos()
    }

    override fun periodic() {
        io.updateInputs(input)
        Logger.processInputs("Launcher", input)
    }

    enum class LaunchType {
        AMP, SPEAKER
    }
}