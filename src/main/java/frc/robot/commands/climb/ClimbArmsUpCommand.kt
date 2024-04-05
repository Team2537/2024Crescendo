package frc.robot.commands.climb

import frc.robot.Constants
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ClimbSubsystem
import lib.near

class ClimbArmsUpCommand : Command() {
    override fun initialize() {
    }

    override fun execute() {
        ClimbSubsystem.armsUp()
    }

    override fun isFinished(): Boolean {
        return ClimbSubsystem.leftMotor.encoder.position.near(
            Constants.ClimbConstants.ARMS_UP_ENCODER_POSITION,
            Constants.ClimbConstants.ARMS_ENCODER_TOLERANCE
        )
    }

    override fun end(interrupted: Boolean) {
        ClimbSubsystem.stop()
    }
}
