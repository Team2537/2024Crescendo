package frc.robot.commands

import frc.robot.Constants
import com.revrobotics.CANSparkBase
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem
import kotlin.math.abs

class ClimbArmsUpCommand : Command() {
    override fun initialize() {
    }

    override fun execute() {
        ClimbSubsystem.armsUp()
    }

    override fun isFinished(): Boolean {
        return abs(ClimbSubsystem.leftMotor.encoder.position - Constants.ClimbConstants.ARMS_UP_ENCODER_POSITION) <= Constants.ClimbConstants.ARMS_ENCODER_TOLERANCE
    }

    override fun end(Interrupted: Boolean) {
        ClimbSubsystem.stop()
    }
}
