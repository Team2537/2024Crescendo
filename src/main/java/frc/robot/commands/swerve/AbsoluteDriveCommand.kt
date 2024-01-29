package frc.robot.commands.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.subsystems.SwerveSubsystem
import swervelib.SwerveController
import swervelib.math.SwerveMath
import java.util.function.DoubleSupplier

class AbsoluteDriveCommand(
    vForwards: DoubleSupplier,
    vStrafe: DoubleSupplier,
    headingX: DoubleSupplier,
    headingY: DoubleSupplier,
) : Command() {
    private val swerveSubsystem = SwerveSubsystem

    val vForwards: DoubleSupplier
    val vStrafe: DoubleSupplier
    val headingX: DoubleSupplier
    val headingY: DoubleSupplier
    var initRotation: Boolean = false

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveSubsystem)

        this.vForwards = vForwards
        this.vStrafe = vStrafe
        this.headingX = headingX
        this.headingY = headingY

    }

    override fun initialize() {
        initRotation = true
    }

    override fun execute() {
        var desiredSpeeds: ChassisSpeeds = swerveSubsystem.getTargetSpeeds(vForwards.asDouble, vStrafe.asDouble, headingX.asDouble, headingY.asDouble)

        if(initRotation){
            if(headingX.asDouble == 0.0 && headingY.asDouble == 0.0){
                var firstLoopHeading: Rotation2d = swerveSubsystem.getHeading()
                desiredSpeeds = swerveSubsystem.getTargetSpeeds(0.0, 0.0, firstLoopHeading.sin, firstLoopHeading.cos)
            }

            initRotation = false
        }

        var translation: Translation2d = SwerveController.getTranslation2d(desiredSpeeds)
        translation = SwerveMath.limitVelocity(translation, swerveSubsystem.getFieldVelocity(),
            swerveSubsystem.getPose(),
            Constants.LOOP_TIME,
            Constants.ROBOT_MASS,
            listOf(Constants.CHASSIS),
            swerveSubsystem.getSwerveDriveConfiguration()
        )

        swerveSubsystem.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
