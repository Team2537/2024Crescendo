package frc.robot.commands.swerve

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import java.util.function.DoubleSupplier
import lib.math.slowmodeFunction

class AngVelocityDrive(
    private val forwardsVelocity: DoubleSupplier,
    private val strafeVelocity: DoubleSupplier,
    private val rotationVelocity: DoubleSupplier,
    private val slowMode: DoubleSupplier
) : Command() {
    private val subsystem = SwerveSubsystem
    private val slowModeFunction = slowmodeFunction
    private val xLimiter = subsystem.controller.xLimiter
    private val yLimiter = subsystem.controller.yLimiter
    private val rotLimiter = subsystem.controller.angleLimiter

    init {
        addRequirements(subsystem)
    }

    override fun execute() {
        val forwardsSpeed = xLimiter.calculate(forwardsVelocity.asDouble) * slowModeFunction(slowMode.asDouble)
        val strafeSpeed = yLimiter.calculate(strafeVelocity.asDouble) * slowModeFunction(slowMode.asDouble)
        val rotationSpeed = rotLimiter.calculate(rotationVelocity.asDouble) * slowModeFunction(slowMode.asDouble)

        subsystem.drive(forwardsSpeed, strafeSpeed, rotationSpeed)
    }

}