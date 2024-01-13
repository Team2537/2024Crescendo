package frc.robot.subsystems

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import swervelib.SwerveDrive
import swervelib.parser.SwerveParser

object SwerveSubsystem : SubsystemBase() {
    private val swerveDrive: SwerveDrive
    var maximumSpeed: Double = Units.feetToMeters(12.0)

    private var autoBuilder: SwerveAutoBuilder? = null

    init {
        try {
            swerveDrive = SwerveParser(Constants.FileConstants.SWERVE_CONFIG).createSwerveDrive(maximumSpeed)
        } catch (e: Exception) {
            throw RuntimeException("Error creating swerve drive", e)
        }

        swerveDrive.setHeadingCorrection(false)
    }

    fun drive(
        translation: Translation2d,
        rotation: Double,
        fieldOriented: Boolean,
    ) {
        swerveDrive.drive(translation, rotation, fieldOriented, false)
    }

    fun drive(
        translation: Translation2d,
        rotation: Double,
        fieldOriented: Boolean,
        centerOfRotation: Translation2d,
    ) {
        swerveDrive.drive(translation, rotation, fieldOriented, false, centerOfRotation)
    }

    fun driveFieldOriented(velocity: ChassisSpeeds) {
        swerveDrive.driveFieldOriented(velocity)
    }

    fun drive(velocity: ChassisSpeeds) {
        swerveDrive.drive(velocity)
    }

    fun getKinematics() = swerveDrive.kinematics

    fun resetOdometry(initialHolonomicPose: Pose2d) {
        swerveDrive.resetOdometry(initialHolonomicPose)
    }

    fun getPose() = swerveDrive.pose

    fun setChassisSpeeds(velocity: ChassisSpeeds) {
        swerveDrive.setChassisSpeeds(velocity)
    }

    fun postTrajectory(trajectory: Trajectory) {
        swerveDrive.postTrajectory(trajectory)
    }

    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    fun setMotorBrake(brake: Boolean) {
        swerveDrive.setMotorIdleMode(brake)
    }

    fun getHeading() = swerveDrive.yaw

    fun getTargetSpeeds(
        vForward: Double,
        vSide: Double,
        angle: Rotation2d,
    ): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(vForward, vSide, angle.radians, getHeading().radians, maximumSpeed)
    }

    fun getFieldVelocity(): ChassisSpeeds? {
        return swerveDrive.fieldVelocity
    }

    fun getRobotVelocity(): ChassisSpeeds? {
        return swerveDrive.robotVelocity
    }

    fun getSwerveController() = swerveDrive.swerveController

    fun getSwerveDriveConfiguration() = swerveDrive.swerveDriveConfiguration

    fun lock() {
        swerveDrive.lockPose()
    }

    fun getPitch() = swerveDrive.pitch

    fun createPathPlannerCommand(
        path: String?,
        constraints: PathConstraints?,
        eventMap: Map<String?, Command?>?,
        translation: PIDConstants?,
        rotation: PIDConstants?,
        useAllianceColor: Boolean,
    ): Command {
        val pathGroup = PathPlanner.loadPathGroup(path, constraints)
        if (autoBuilder == null) {
            autoBuilder =
                SwerveAutoBuilder(
                    { swerveDrive.getPose() }, { pose: Pose2d? -> swerveDrive.resetOdometry(pose) },
                    translation,
                    rotation, { chassisSpeeds: ChassisSpeeds? -> swerveDrive.setChassisSpeeds(chassisSpeeds) },
                    eventMap,
                    useAllianceColor,
                    this,
                )
        }
        return autoBuilder!!.fullAuto(pathGroup)
    }
}
