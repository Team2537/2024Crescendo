import com.ctre.phoenix.sensors.WPI_Pigeon2
import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.util.SingletonXboxController
import swervelib.SwerveDrive
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry


object SwerveSubsystem : SubsystemBase() {
    private val swerveDrive: SwerveDrive
    private var autoBuilder: SwerveAutoBuilder? = null
    private val controller = SingletonXboxController
    private var swerveTab: ShuffleboardTab

    init {
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH
        try {
            swerveDrive = SwerveParser(Constants.FileConstants.SWERVE_CONFIG).createSwerveDrive()
        } catch (e: Exception) {
            throw RuntimeException("Failed to create swerve drive", e)
        }

        swerveTab = Shuffleboard.getTab("Swerve")

        setMotorBrake(true)
    }

    fun drive(translation2d: Translation2d, rotation: Double, fieldOriented: Boolean) {
        swerveDrive.drive(translation2d, rotation, fieldOriented, false)
    }

    fun getKinematics() = swerveDrive.kinematics

    fun resetOdometry(initalHolonomicPose: Pose2d){
        swerveDrive.resetOdometry(initalHolonomicPose)
    }

    fun getPose(): Pose2d {
        return swerveDrive.getPose()
    }

    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds){
        swerveDrive.setChassisSpeeds(chassisSpeeds)
    }

    fun postTrajectory(trajectory: Trajectory){
        swerveDrive.postTrajectory(trajectory)
    }

    fun zeroGyro(){
        swerveDrive.zeroGyro()
    }

    fun setMotorBrake(brake: Boolean){
        swerveDrive.setMotorIdleMode(brake)
    }

    fun getHeading(): Rotation2d {
        return swerveDrive.yaw
    }

    fun getTargetSpeeds(vForward: Double, vSide: Double, angle: Rotation2d): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(vForward, vSide, angle.radians, getHeading().radians)
    }

    fun getFieldVelocity(): ChassisSpeeds {
        return swerveDrive.fieldVelocity
    }

    fun getRobotVelocity(): ChassisSpeeds {
        return swerveDrive.robotVelocity
    }

    fun getSwerveController() = swerveDrive.swerveController

    fun getSwerveDriveConfiguration(): SwerveDriveConfiguration? {
        return swerveDrive.swerveDriveConfiguration
    }

    fun lock() {
        swerveDrive.lockPose()
    }

    fun getPitch(): Rotation2d? {
        return swerveDrive.pitch
    }

    fun createPathPlannerCommand(
        path: String?, constraints: PathConstraints?, eventMap: Map<String?, Command?>?,
        translation: PIDConstants?, rotation: PIDConstants?, useAllianceColor: Boolean
    ): Command {
        val pathGroup = PathPlanner.loadPathGroup(path, constraints)
        if (autoBuilder == null) {
            autoBuilder =
                SwerveAutoBuilder({ swerveDrive.getPose() }, { pose: Pose2d? -> swerveDrive.resetOdometry(pose) },
                    translation,
                    rotation, { chassisSpeeds: ChassisSpeeds? -> swerveDrive.setChassisSpeeds(chassisSpeeds) },
                    eventMap,
                    useAllianceColor,
                    this
                )
        }
        return autoBuilder!!.fullAuto(pathGroup)
    }



}