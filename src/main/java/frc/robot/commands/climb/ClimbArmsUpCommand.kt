package frc.robot.commands

import com.revrobotics.CANSparkBase
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem
import kotlin.math.abs

class ClimbArmsUpCommand : Command() {
    override fun initialize() {
    }

    override fun execute() {
        ClimbSubsystem.leftMotor.set(.3)
        ClimbSubsystem.rightMotor.set(.3)
    }

    override fun isFinished(): Boolean {
        //return abs(ClimbSubsystem.leftMotor.encoder.position - 1.0) <= 0.1
        //return abs(ClimbSubsystem.rightMotor.encoder.position - 1.0) <= 0.1
        return false
    }

    override fun end(Interrupted: Boolean) {
        ClimbSubsystem.leftMotor.set(0.0);
        ClimbSubsystem.rightMotor.set(0.0);
    }
}
