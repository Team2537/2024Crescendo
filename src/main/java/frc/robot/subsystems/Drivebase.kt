package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.util.GeometryUtil
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.util.Units as OldUnits
import edu.wpi.first.units.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.Constants
import lib.evilGetHeading
import lib.math.units.into
import lib.vision.VisionMeasurement
import swervelib.SwerveDrive
import swervelib.SwerveDriveTest
import swervelib.SwerveModule
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
class Drivebase : SubsystemBase() {

    /** The swerve drive object */
    private val swerveDrive: SwerveDrive

    /** The maximum speed of the swerve drive */
    var maximumSpeed: Double = OldUnits.feetToMeters(14.5)

    /** The Shuffleboard tab for the SwerveSubsystem */
    var tab: ShuffleboardTab = Shuffleboard.getTab("Testing")

    /** SwerveModuleStates publisher for swerve display */
    var swerveStates: StructArrayPublisher<SwerveModuleState> = NetworkTableInstance.getDefault().
        getStructArrayTopic("SwerveStates/swerveStates", SwerveModuleState.struct).publish()

    /**
     * PID Controller for the heading of the robot.
     * Currently not used, as feedforward is a much better solution to fix drift.
     */
    val HeadingPID: PIDController = PIDController(0.005, 0.01, 0.0)


    /**
     * Whether the robot should drive field oriented or robot oriented.
     * @see drive
     */
    var fieldOriented: Boolean = true



    init {

        // Set the verbosity of the telemetry, HIGH for debugging, LOW for competition
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH

        // Create the swerve drive object from a parser
        // Potentially use the RIOs serial number to determine which configuration to use
        try {
            swerveDrive = SwerveParser(Constants.FileConstants.CRESCENDO_CONFIG).createSwerveDrive(maximumSpeed)
        } catch (e: Exception) {
            e.printStackTrace()
            throw RuntimeException("Error creating swerve drive", e)
        }

        // Set whether heading correction from YAGSL should be enabled
        swerveDrive.setHeadingCorrection(true)

        // Make sure no modules are in anti-jitter mode, untested and it does funny stuff with the encoder offsets
        swerveDrive.modules.forEach {
            it.setAntiJitter(false)
        }

        // Set whether the cosine compensator should be enabled, as it breaks simulation
        if (SwerveDriveTelemetry.isSimulation) {
            swerveDrive.setCosineCompensator(false)
        } else {
            swerveDrive.setCosineCompensator(true)
        }

        // Make sure brake mode is on
        setMotorBrake(true)

        // Set up the PathPlanner stuff, see the actual method
        configurePathPlanner()

        // Lots and lots of logging
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
                PIDConstants(1.0, 0.0, 1.0),
                PIDConstants(1.0, 0.0, 0.0),
                4.0,
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

    /**
     * Directly send voltage to the drive motors.
     * @param volts The voltage to send to the motors.
     */
    fun setRawMotorVoltage(volts: Double){
        swerveDrive.modules.forEach {
            it.driveMotor.voltage = volts
        }
    }

    /**
     * Get a SysIdRoutine for the drive motors.
     * @see SysIdRoutine
     * @return A custom SysIdRoutine for the drive motors.
     */
    fun getDriveSysIDRoutine(): SysIdRoutine {
        return SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                { volts: Measure<Voltage> ->
                    swerveDrive.modules.forEach {
                        it.driveMotor.voltage = volts into Units.Volt
                    }
                },
                { log: SysIdRoutineLog ->
                    swerveDrive.modules.forEach {
                        logDriveMotor(it, log)
                    }
                },
                this
            )
        )
    }

    /**
     * Generate a full command to SysID the drive motors
     * @return A command that SysIDs the drive motors.
     */
    fun getDriveSysIDCommand(): Command {
        return SequentialCommandGroup(
            getDriveSysIDRoutine().dynamic(SysIdRoutine.Direction.kForward),
            WaitCommand(1.0),
            getDriveSysIDRoutine().dynamic(SysIdRoutine.Direction.kReverse),
            WaitCommand(1.0),
            getDriveSysIDRoutine().quasistatic(SysIdRoutine.Direction.kForward),
            WaitCommand(1.0),
            getDriveSysIDRoutine().quasistatic(SysIdRoutine.Direction.kReverse)
        )
    }

    /**
     * Logging function to easily log for SysID.
     * @see SysIdRoutineLog
     * @param module The module to log.
     * @param log The SysIdRoutineLog to log to.
     */
    private fun logDriveMotor(module: SwerveModule, log: SysIdRoutineLog){
        log.motor(module.configuration.name)
            .voltage(Units.Volt.of(module.driveMotor.voltage))
            .linearPosition(Units.Meters.of(module.driveMotor.position))
            .linearVelocity(Units.MetersPerSecond.of(module.driveMotor.velocity))
    }

    /**
     * Calculate the heading PID for the robot.
     * @param measurement The current heading of the robot.
     * @param setpoint The desired heading of the robot.
     * @return The calculated output of the PID controller.
     */
    @Deprecated("Use feedforward instead")
    fun calculateHeadingPID(measurement: Double, setpoint: Double): Double {
        return HeadingPID.calculate(measurement, setpoint)
    }

    /**
     * Return SysID command for drive motors from YAGSL
     * @return A command that SysIDs the drive motors.
     */
    fun sysIdDriveMotor(): Command? {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                SysIdRoutine.Config(),
                this,
                swerveDrive, 12.0),
            3.0, 5.0, 3.0
            )
    }

    /**
     * Return SysID command for angle motors from YAGSL
     * @return A command that SysIDs the angle motors.
     */
    fun sysIdAngleMotorCommand(): Command {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(
                SysIdRoutine.Config(),
                this, swerveDrive
            ),
            3.0, 5.0, 3.0
        )
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
        var startPosition: Pose2d = Pose2d()
        if(PathPlannerAuto.getStaringPoseFromAutoFile(autoName) == null) {
            startPosition = PathPlannerAuto.getPathGroupFromAutoFile(autoName)[0].startingDifferentialPose
        } else {
            startPosition = PathPlannerAuto.getStaringPoseFromAutoFile(autoName)
        }

        if(DriverStation.getAlliance() == Optional.of(Alliance.Red)){
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

    /**
     * Method to set the desired speeds of the swerve drive.
     * @param chassisSpeeds The desired speeds of the swerve drive.
     */
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

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and angle X and Y components.
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param headingX The desired X component of the angle.
     * @param headingY The desired Y component of the angle.
     * @return The generated ChassisSpeeds object.
     */
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

    /**
     * Add a vision measurement to the swerve drive's pose estimator.
     * @param measurement The pose measurement to add.
     * @param timestamp The timestamp of the pose measurement.
     */
    fun addVisionMeasurement(measurement: Pose2d, timestamp: Double) {
        swerveDrive.addVisionMeasurement(measurement, timestamp)
    }

    /**
     * Add a vision measurement to the swerve drive's pose estimator.
     * @see VisionMeasurement
     * @param measurement The vision measurement to add, contains its own pose and timestamp.
     */
    fun addVisionMeasurement(measurement: VisionMeasurement) {
        swerveDrive.addVisionMeasurement(measurement.position.toPose2d(), measurement.timestamp)
    }

    /**
     * Set the standard deviations of the vision measurements.
     * @param stdDevX The standard deviation of the X component of the vision measurements.
     * @param stdDevY The standard deviation of the Y component of the vision measurements.
     * @param stdDevTheta The standard deviation of the rotational component of the vision measurements.
     */
    fun setVisionMeasurementStdDevs(stdDevX: Double, stdDevY: Double, stdDevTheta: Double) {
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevX, stdDevY, stdDevTheta))
    }

    /** @suppress */
    override fun periodic() {
        swerveStates.set(swerveDrive.states)
    }

    /** function to toggle field oriented drive */
    fun toggleFieldOriented() {
        fieldOriented = !fieldOriented
    }
}
