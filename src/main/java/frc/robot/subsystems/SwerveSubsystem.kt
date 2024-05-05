package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.util.GeometryUtil
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Constants
import org.littletonrobotics.junction.Logger
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.SwerveDriveTest
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File
import java.util.*
import edu.wpi.first.math.util.Units as Conversions

/**
 * The subsystem that controls the swerve drive.
 *
 * @constructor Creates a SwerveSubsystem from JSON Configuration Files.
 * @author Falon Clark
 * @since 1/15/2024
 */
object SwerveSubsystem : SubsystemBase() {
    private val drivebase: SwerveDrive
    private val directory: File = Constants.FileConstants.CRESCENDO_CONFIG

    val forwardsSlewRateLimiter: SlewRateLimiter = SlewRateLimiter(2.0)
    val strafeSlewRateLimiter: SlewRateLimiter = SlewRateLimiter(2.0)
    val rotationSlewRateLimiter: SlewRateLimiter = SlewRateLimiter(2.0)

    private var shouldFieldOriented: Boolean = true

    val maxSpeed: Double = Conversions.feetToMeters(14.5)

    val kinematics: SwerveDriveKinematics
        get() = drivebase.kinematics

    val pose: Pose2d
        get() = drivebase.pose

    val heading: Rotation2d
        get() = pose.rotation

    val fieldVelocity: ChassisSpeeds
        get() = drivebase.fieldVelocity

    val robotVelocity: ChassisSpeeds
        get() = drivebase.robotVelocity

    val controller: SwerveController
        get() = drivebase.swerveController

    val config: SwerveDriveConfiguration
        get() = drivebase.swerveDriveConfiguration

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH
        try {
            drivebase = SwerveParser(directory).createSwerveDrive(maxSpeed)
        } catch (e: Exception) {
            DriverStation.reportError("Failed to create swerve drive from config file", false)
            e.printStackTrace()
            throw e
        }

        drivebase.setHeadingCorrection(false)
        drivebase.setCosineCompensator(!SwerveDriveTelemetry.isSimulation)

        controller.addSlewRateLimiters(
            forwardsSlewRateLimiter,
            strafeSlewRateLimiter,
            rotationSlewRateLimiter
        )

        drivebase.resetDriveEncoders()
    }

    /**
     * Configures PathPlanner's AutoBuilder.
     * @see AutoBuilder
     */
    fun configurePathPlanner()  {
        // TODO: Configure path planner's AutoBuilder
        AutoBuilder.configureHolonomic(
            this::pose,
            this::resetOdometry,
            this::robotVelocity,
            this::setChassisSpeeds,
            HolonomicPathFollowerConfig(
                PIDConstants(1.0, 0.0, 1.0),
                PIDConstants(1.0, 0.0, 0.0),
                4.0,
                config.driveBaseRadiusMeters,
                ReplanningConfig(
                    true,
                    true,
                )
            ),
            {
                if (DriverStation.getAlliance().isPresent){
                    DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                } else {
                    false
                }
            },
            this

        )
    }

    fun getAutonomousCommand(
        autoName: String,
        setOdomAtStart: Boolean,
    ): Command {
        var startPosition: Pose2d = Pose2d()
        if(PathPlannerAuto.getStaringPoseFromAutoFile(autoName) == null) {
            startPosition = PathPlannerAuto.getPathGroupFromAutoFile(autoName)[0].startingDifferentialPose
        } else {
            startPosition = PathPlannerAuto.getStaringPoseFromAutoFile(autoName)
        }

        if(DriverStation.getAlliance() == Optional.of(DriverStation.Alliance.Red)){
            startPosition = GeometryUtil.flipFieldPose(startPosition)
        }

        if (setOdomAtStart)
        {
            if (startPosition != null) {
                resetOdometry(startPosition)
            }
        }

        // TODO: Configure path planner's AutoBuilder
        return PathPlannerAuto(autoName)
    }

    fun characterizeDrive(): Command? {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                SysIdRoutine.Config(), this, drivebase, 12.0
            ), 3.0, 5.0, 3.0
        )
    }

    fun characterizeAngle(): Command? {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(
                SysIdRoutine.Config(), this, drivebase
            ), 3.0, 5.0, 3.0
        )
    }

    fun drive(xSpeed: Double, ySpeed: Double, rot: Double) {
        drivebase.drive(Translation2d(xSpeed, ySpeed), rot, shouldFieldOriented, false)
    }

    fun drive(translation2d: Translation2d, rot: Double) {
        drivebase.drive(translation2d, rot, shouldFieldOriented, false)
    }

    fun drive(xSpeed: Double, ySpeed: Double, rot: Double, center: Translation2d) {
        drivebase.drive(Translation2d(xSpeed, ySpeed), rot, shouldFieldOriented, false, center)
    }

    fun drive(velocity: ChassisSpeeds) {
        drivebase.drive(velocity)
    }

    fun driveFieldOriented(velocity: ChassisSpeeds) {
        drivebase.driveFieldOriented(velocity)
    }

    fun resetOdometry(intialHolonomicPose: Pose2d) {
        drivebase.resetOdometry(intialHolonomicPose)
    }

    fun setChassisSpeeds(speeds: ChassisSpeeds) {
        drivebase.setChassisSpeeds(speeds)
    }

    fun postTrajectory(trajectory: Trajectory) {
        drivebase.postTrajectory(trajectory)
    }

    fun zeroGyro() {
        drivebase.zeroGyro()
    }

    fun setMotorBrakeMode(brake: Boolean) {
        drivebase.setMotorIdleMode(brake)
    }

    fun lock() {
        drivebase.lockPose()
    }

    fun toggleFieldOriented() {
        shouldFieldOriented = !shouldFieldOriented
    }

    fun getTargetSpeeds(xSpeed: Double, ySpeed: Double, rot: Double): ChassisSpeeds {
        return controller.getRawTargetSpeeds(xSpeed, ySpeed, rot)
    }

    override fun periodic() {
//        Logger.recordOutput("Swerve/Pose", pose)
//        Logger.recordOutput("Swerve/Field Velocity", fieldVelocity)
//        Logger.recordOutput("Swerve/Robot Velocity", robotVelocity)
//        Logger.recordOutput("Swerve/Heading", heading)
//        Logger.recordOutput("Swerve/Field Oriented", shouldFieldOriented)
    }

}
