package frc.robot.commands.swerve

import edu.wpi.first.wpilibj2.command.CommandBase
import SwerveSubsystem
import edu.wpi.first.math.geometry.Translation2d
import swervelib.SwerveController
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.contracts.contract

class TeleopDrive(vForward: DoubleSupplier, vSide: DoubleSupplier, omega: DoubleSupplier,
                  driveMode: BooleanSupplier, isOpenLoop: Boolean, slowMode: BooleanSupplier) : CommandBase() {
    private val swerveSubsystem = SwerveSubsystem

    private val vForward: DoubleSupplier
    private val vSide: DoubleSupplier
    private val omega: DoubleSupplier
    private val driveMode: BooleanSupplier
    private val isOpenLoop: Boolean
    private val slowMode: BooleanSupplier

    private val swerveController: SwerveController


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)

        this.vForward = vForward
        this.vSide = vSide
        this.omega = omega
        this.driveMode = driveMode
        this.isOpenLoop = isOpenLoop
        this.slowMode = slowMode
        this.swerveController = swerveSubsystem.getSwerveController()



    }

    override fun initialize() {}

    override fun execute() {
        var xVelocity = vForward.getAsDouble()
        var yVelocity = vSide.getAsDouble()
        var angleVelocity = omega.getAsDouble()

        if(slowMode.asBoolean){
            xVelocity *= 0.6
            yVelocity *= 0.6
            angleVelocity *= 0.6
        }

        var speed: Translation2d = Translation2d(xVelocity * swerveController.config.maxSpeed , yVelocity * swerveController.config.maxSpeed)

        swerveSubsystem.drive(speed, angleVelocity * swerveController.config.maxAngularVelocity, driveMode.asBoolean)


    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
