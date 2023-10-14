package frc.robot.subsystems


import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

import swervelib.SwerveDrive
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File
import kotlin.math.pow


object SwerveSubsystem : SubsystemBase() {
    lateinit var drivebase: SwerveDrive
    private var swerveAutoBuilder: SwerveAutoBuilder? = null;

    private val directory: File = File(Filesystem.getDeployDirectory(), "swerve")

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH
        try {
            drivebase = SwerveParser(directory).createSwerveDrive()
        } catch (e: Exception) {
            throw RuntimeException(e)
        }
    }

    fun drive(translation: Translation2d, rotation: Double, fieldOriented: Boolean, isOpenLoop: Boolean) {
        drivebase.drive(translation, rotation, fieldOriented, isOpenLoop)
    }

    fun getKinematics(): SwerveDriveKinematics = drivebase.kinematics

    fun resetOdometry(initialHolonomicPose: Pose2d) {
        drivebase.resetOdometry(initialHolonomicPose)
    }

    fun getPose(): Pose2d = drivebase.getPose()

    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        drivebase.setChassisSpeeds(chassisSpeeds)
    }

    fun postTrajectory(trajectory: Trajectory){
        drivebase.postTrajectory(trajectory)
    }

    fun zeroGyro(){
        drivebase.zeroGyro()
    }

    fun setMotorBrake(brake: Boolean){
        drivebase.setMotorIdleMode(brake)
    }

    fun getHeading(): Rotation2d = drivebase.getYaw()

    fun getTargetSpeeds(xInput: Double, yInput: Double, headingX: Double, headingY: Double): ChassisSpeeds {
        var xInput = xInput
        var yInput = yInput
        xInput = xInput.pow(3.0)
        yInput = yInput.pow(3.0)
        return drivebase.swerveController.getTargetSpeeds(
            xInput,
            yInput,
            headingX,
            headingY,
            getHeading().getRadians()
        )
    }

    fun getTargetSpeeds(xInput: Double, yInput: Double, angle: Rotation2d): ChassisSpeeds {
        var xInput = xInput
        var yInput = yInput
        xInput = xInput.pow(3.0)
        yInput = yInput.pow(3.0)
        return drivebase.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians())
    }

    fun getFieldVelocity(): ChassisSpeeds = drivebase.getFieldVelocity()

    fun getRobotVelocity(): ChassisSpeeds = drivebase.getRobotVelocity()

    fun getSwerveController() = drivebase.swerveController

    fun getSwerveDriveConfiguration(): SwerveDriveConfiguration {
        return drivebase.swerveDriveConfiguration
    }

    fun lock() {
        drivebase.lockPose()
    }

    fun getPitch(): Rotation2d {
        return drivebase.getPitch()
    }

    fun createPathPlannerCommand(
        path: String, constraints: PathConstraints, eventMap: Map<String, Command>,
        translation: PIDConstants, rotation: PIDConstants, useAllianceColor: Boolean
    ): Command {
        val pathGroup = PathPlanner.loadPathGroup(path, constraints)
        if (swerveAutoBuilder == null) {
            swerveAutoBuilder = SwerveAutoBuilder(
                drivebase::getPose,
                drivebase::resetOdometry,
                translation,
                rotation,
                drivebase::setChassisSpeeds,
                eventMap,
                useAllianceColor,
                this
            )
        }
        return swerveAutoBuilder!!.fullAuto(pathGroup)
    }

}