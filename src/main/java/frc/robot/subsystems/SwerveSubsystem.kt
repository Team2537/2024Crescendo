package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.util.GeometryUtil
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import lib.evilGetHeading
import lib.vision.Limelight
import lib.vision.VisionMeasurement
import swervelib.SwerveDrive
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.util.*

/**
 * The subsystem that controls the swerve drive.
 *
 * @constructor Creates a SwerveSubsystem from JSON Configuration Files.
 * @author Falon Clark
 * @since 1/15/2024
 */
object SwerveSubsystem : SubsystemBase() {
    private val swerveDrive: SwerveDrive

    /** @suppress */
    var maximumSpeed: Double = Units.feetToMeters(14.5)
    var tab: ShuffleboardTab = Shuffleboard.getTab("Testing")
    var swerveStates: StructArrayPublisher<SwerveModuleState> = NetworkTableInstance.getDefault().
        getStructArrayTopic("SwerveStates/swerveStates", SwerveModuleState.struct).publish()

    val HeadingPID: PIDController = PIDController(0.005, 0.01, 0.0)

    /** True is field oriented driving */
    var fieldOriented: Boolean = true
    val gamepieceLimelight: Limelight = Limelight("limelight-intake", Pose3d())



    init {

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH

        try {
            swerveDrive = SwerveParser(Constants.FileConstants.CRESCENDO_CONFIG).createSwerveDrive(maximumSpeed)
        } catch (e: Exception) {
            e.printStackTrace()
            throw RuntimeException("Error creating swerve drive", e)
            throw RuntimeException("Error creating swerve drive", e)
        }

        swerveDrive.setHeadingCorrection(true)
        swerveDrive.modules.forEach {
            it.setAntiJitter(false)
        }

//        swerveDrive.setCosineCompensator(false)

        setMotorBrake(true)

        configurePathPlanner()

        tab.addDouble("Heading") { getHeading().radians }

        tab.addDouble("Front Left Velocity") { Math.abs(swerveDrive.modules[0].driveMotor.velocity) }
        tab.addDouble("Front Right Velocity") { Math.abs(swerveDrive.modules[1].driveMotor.velocity) }
        tab.addDouble("Back Left Velocity") { Math.abs(swerveDrive.modules[2].driveMotor.velocity) }
        tab.addDouble("Back Right Velocity") { Math.abs(swerveDrive.modules[3].driveMotor.velocity) }

        tab.addDouble("Front Left Voltage") { Math.abs(swerveDrive.modules[0].driveMotor.voltage) }
        tab.addDouble("Front Right Voltage") { Math.abs(swerveDrive.modules[1].driveMotor.voltage) }
        tab.addDouble("Back Left Voltage") { Math.abs(swerveDrive.modules[2].driveMotor.voltage) }
        tab.addDouble("Back Right Voltage") { Math.abs(swerveDrive.modules[3].driveMotor.voltage)}

        tab.addDouble("Evil heading") { swerveDrive.evilGetHeading() }
    }

    /**
     * Configures PathPlanner's AutoBuilder.
     * @see AutoBuilder
     */
    fun configurePathPlanner()  {
        // TODO: Configure path planner's AutoBuilder
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
            this::setChassisSpeeds,
            HolonomicPathFollowerConfig(
                PIDConstants(5.0, 0.0, 1.0),
                PIDConstants(10.0, 0.0, 0.0),
                2.7,
                swerveDrive.swerveDriveConfiguration.driveBaseRadiusMeters,
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

    fun calculateHeadingPID(measurement: Double, setpoint: Double): Double {
        return HeadingPID.calculate(measurement, setpoint)
    }



    /**
     * Gets a command that follows a path created in PathPlanner.
     * @param pathName The path's file name.
     * @param setOdomAtStart Whether to update the robot's odometry to the start pose of the path.
     * @return A command that follows the path.
     */
    fun getAutonomousCommand(
        autoName: String,
        setOdomAtStart: Boolean,
    ): Command {
        var startPosition: Translation2d = PathPlannerAuto.getPathGroupFromAutoFile(autoName)[0].getPoint(0).position
        var pose: Pose2d = Pose2d(startPosition, getHeading())


        if(DriverStation.getAlliance() == Optional.of(Alliance.Red)){
            pose = GeometryUtil.flipFieldPose(pose)
        }

        if (setOdomAtStart)
            {
                resetOdometry(pose)
            }

        // TODO: Configure path planner's AutoBuilder
        return PathPlannerAuto(autoName)
    }

    /**
     * Simple drive method that translates and rotates the robot.
     * @param translation The desired X and Y velocity of the robot.
     * @param rotation The desired rotational velocity of the robot.
     * @param fieldOriented Whether the robot's motion should be field oriented or robot oriented.
     */
    fun drive(
        translation: Translation2d,
        rotation: Double,
        fieldOriented: Boolean,
    ) {
        swerveDrive.drive(translation, rotation, fieldOriented, false)
    }

    /**
     * Advanced drive method that translates and rotates the robot, with a custom center of rotation.
     * @param translation The desired X and Y velocity of the robot.
     * @param rotation The desired rotational velocity of the robot.
     * @param fieldOriented Whether the robot's motion should be field oriented or robot oriented.
     * @param centerOfRotation The center of rotation of the robot.
     */
    fun drive(
        translation: Translation2d,
        rotation: Double,
        fieldOriented: Boolean,
        centerOfRotation: Translation2d,
    ) {
        swerveDrive.drive(translation, rotation, fieldOriented, false, centerOfRotation)
    }

    /**
     * Simple drive method that uses ChassisSpeeds to control the robot.
     * @param velocity The desired ChassisSpeeds of the robot
     */
    fun drive(velocity: ChassisSpeeds) {
        swerveDrive.drive(velocity)
    }

    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds)
    }

    /**
     * Method to get the Kinematics object of the swerve drive.
     */
    fun getKinematics() = swerveDrive.kinematics

    /**
     * Method to reset the odometry of the robot to a desired pose.
     * @param initialHolonomicPose The desired pose to reset the odometry to.
     */
    fun resetOdometry(initialHolonomicPose: Pose2d) {
        swerveDrive.resetOdometry(initialHolonomicPose)
    }

    /**
     * Method to get the current pose of the robot.
     * @return The current pose of the robot.
     */
    fun getPose() = swerveDrive.pose

    /**
     * Method to display a desired trajectory to a field2d object.
     */
    fun postTrajectory(trajectory: Trajectory) {
        swerveDrive.postTrajectory(trajectory)
    }

    /**
     * Method to zero the gyro.
     */
    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    /**
     * Method to toggle the motor's brakes.
     * @param brake Whether to set the motor's brakes to true or false.
     */
    fun setMotorBrake(brake: Boolean) {
        swerveDrive.setMotorIdleMode(brake)
    }

    /**
     * Method to get the current heading of the robot.
     * @return The current heading of the robot.
     */
    fun getHeading() = swerveDrive.yaw

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and Rotational velocity.
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param angle The desired rotational velocity of the robot.
     * @return The generated ChassisSpeeds object.
     */
    fun getTargetSpeeds(
        vForward: Double,
        vSide: Double,
        angle: Rotation2d,
    ): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(vForward, vSide, angle.radians, getHeading().radians, maximumSpeed)
    }

    fun getTargetSpeeds(
        vForward: Double,
        vSide: Double,
        headingX: Double,
        headingY: Double
    ): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(vForward, vSide, headingX, headingY, getHeading().radians, maximumSpeed)
    }

    /**
     * Method to get the current field oriented velocity of the robot.
     * @return The current field oriented velocity of the robot.
     */
    fun getFieldVelocity(): ChassisSpeeds? {
        return swerveDrive.fieldVelocity
    }

    /**
     * Method to get the current robot oriented velocity of the robot.
     * @return The current robot oriented velocity of the robot.
     */
    fun getRobotVelocity(): ChassisSpeeds? {
        return swerveDrive.robotVelocity
    }

    /**
     * Method to get the SwerveController object of the swerve drive.
     * @return The SwerveController object of the swerve drive.
     */
    fun getSwerveController() = swerveDrive.swerveController

    /**
     * Method to get the SwerveDriveConfiguration object of the swerve drive.
     * @return The SwerveDriveConfiguration object of the swerve drive.
     */
    fun getSwerveDriveConfiguration() = swerveDrive.swerveDriveConfiguration

    /**
     * Method to toggle the lock position of the swerve drive to prevent motion.
     */
    fun lock() {
        swerveDrive.lockPose()
    }

    /**
     * Method to get the current pitch of the robot.
     */
    fun getPitch() = swerveDrive.pitch

    fun addVisionMeasurement(measurement: Pose2d, timestamp: Double) {
        swerveDrive.addVisionMeasurement(measurement, timestamp)
    }

    fun addVisionMeasurement(measurement: VisionMeasurement) {
        swerveDrive.addVisionMeasurement(measurement.position.toPose2d(), measurement.timestamp)
    }

    override fun periodic() {
        swerveStates.set(swerveDrive.states)
    }

    fun toggleFieldOriented() {
        fieldOriented = !fieldOriented
    }
}
