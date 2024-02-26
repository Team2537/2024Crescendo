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
        ClimbSubsystem.leftMotor.set(-0.3)
        ClimbSubsystem.rightMotor.set(-0.3)
    }

    override fun isFinished(): Boolean {
        return false
    }

    override fun end(Interrupted: Boolean) {
        ClimbSubsystem.leftMotor.set(0.0);
        ClimbSubsystem.rightMotor.set(0.0);
    }
}
