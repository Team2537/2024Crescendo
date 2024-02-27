package frc.robot.commands

import com.revrobotics.CANSparkBase
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.subsystems.ClimbSubsystem
import kotlin.math.abs

class PidDownCommand: Command() {
    override fun initialize() {
    }

    override fun execute() {
        ClimbSubsystem.armsDown()
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(Interrupted: Boolean) {
        ClimbSubsystem.stop()
    }
}
