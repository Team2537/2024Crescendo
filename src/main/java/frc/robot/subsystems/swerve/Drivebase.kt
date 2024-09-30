package frc.robot.subsystems.swerve

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2
import frc.robot.subsystems.swerve.gyro.GyroIOSim
import frc.robot.subsystems.swerve.module.ModuleIO
import frc.robot.subsystems.swerve.module.SwerveModule
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import edu.wpi.first.math.util.Units as Conversions

class Drivebase : SubsystemBase("Drivebase") {

    private var driverOrientation: Rotation2d = Rotation2d()
    private var hasAppliedOffset: Boolean = false

    /**
     * Gyro IO for interacting with a gyroscope, automatically initializes between Real, Sim, and Replay (blank interface)
     */
    private val gyro: GyroIO = when (Constants.RobotConstants.mode) {
        Constants.RobotConstants.Mode.REAL -> GyroIOPigeon2(id = 13)
        Constants.RobotConstants.Mode.SIM -> GyroIOSim(this::robotRelativeSpeeds)
        Constants.RobotConstants.Mode.REPLAY -> object : GyroIO {}
    }

    /**
     * Gyro inputs for reading values out of the gyro IO interface
     */
    val gyroInputs: GyroIO.GyroInputs = GyroIO.GyroInputs()

    private val modules: Array<SwerveModule> = arrayOf(
        SwerveModule(flConfig),
        SwerveModule(frConfig),
        SwerveModule(blConfig),
        SwerveModule(brConfig),
    )

    /**
     * Array of [SwerveModuleState] used for storing desired states, these are then logged for tuning purposes
     */
    private val desiredStates: Array<SwerveModuleState> = Array(4) { SwerveModuleState() }

    /**
     * Array of [SwerveModuleState] used for storing measured states from drivebase's modules, these are then logged for tuning purposes
     */
    private val measuredStates: Array<SwerveModuleState> = Array(4) { SwerveModuleState() }

    /**
     * Array of [Translation2d] used for storing the physical positions of modules, used for kinematics
     */
    private val moduleTranslations: Array<Translation2d> = arrayOf(
        Translation2d(9.7859, 9.7859),
        Translation2d(9.7859, -9.7859),
        Translation2d(-9.7859, 9.7859),
        Translation2d(-9.7859, -9.7859),
    )

    /**
     * Array of [SwerveModulePosition] used for storing the distance traveled of each module, this is for odometry
     */
    private val modulePositions: Array<SwerveModulePosition> = Array(4) { SwerveModulePosition() }

    /**
     * Robot relative speeds of the robot, used for logging and gyro simulation
     */
    val robotRelativeSpeeds: ChassisSpeeds
        get() = ChassisSpeeds.fromFieldRelativeSpeeds(
            kinematics.toChassisSpeeds(*measuredStates),
            gyroInputs.yaw.unaryMinus(),
        )

    /**
     * Field relative speeds of the robot, used for logging and gyro simulation
     */
    val fieldRelativeSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(*measuredStates)

    /**
     * Kinematics object used for calculating module states from chassis speeds and vice versa
     */
    private val kinematics: SwerveDriveKinematics = SwerveDriveKinematics(*moduleTranslations)


    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        gyroInputs.yaw,
        getModulePositions(),
        Pose2d(),
    )

    val pose: Pose2d
        get() = poseEstimator.estimatedPosition

    private val driveSysID: SysIdRoutine = SysIdRoutine(
        SysIdRoutine.Config(),
        SysIdRoutine.Mechanism(
            { volts: Measure<Voltage> ->
                modules.forEach {
                    it.pointAt(Rotation2d())
                    it.applyVoltage(volts.into(Units.Volts))
                }
            },
            null,
            this
        )
    )

    private val routineToUse = driveSysID

    /**
     * Method for getting the module positions from the module constants
     *
     * @return Array of [SwerveModulePosition] containing the module positions
     */
    private fun getModulePositions(): Array<SwerveModulePosition> {
        modules.forEachIndexed { index, module ->
            modulePositions[index] = module.modulePosition
        }

        return modulePositions
    }


    /**
     * Method for resetting the heading of the robot
     */
    fun resetHeading() {
        gyro.setYaw(0.0)
    }

    /**
     * Method for setting the heading of the robot
     *
     * @param heading New heading of the robot
     */
    fun applyChassisSpeeds(speeds: ChassisSpeeds) {
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)

        val swerveModuleStates = kinematics.toSwerveModuleStates(discreteSpeeds)

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed)

        Logger.recordOutput("swerve/appliedSpeeds", speeds)

        modules.forEachIndexed { index, module ->
            desiredStates[index] = swerveModuleStates[index]
            module.apply(swerveModuleStates[index])
        }
    }

    /**
     * Factory for creating a command to drive the robot
     * This command will create chassis speeds from the inputs and apply them to the robot
     *
     * @param forwards Supplier for the forwards speed of the robot
     * @param strafe Supplier for the strafe speed of the robot
     * @param rotation Supplier for the rotation speed of the robot
     * @param isFieldOriented Supplier for whether the robot is field oriented or not
     *
     * @return Command for driving the robot
     */
    fun driveCommand(
        forwards: DoubleSupplier,
        strafe: DoubleSupplier,
        rotation: DoubleSupplier,
        isFieldOriented: BooleanSupplier,
    ): Command? {
        return this.run {
            val speeds = if (isFieldOriented.asBoolean) {
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    forwards.asDouble * (maxSpeed into Units.MetersPerSecond),
                    strafe.asDouble * (maxSpeed into Units.MetersPerSecond),
                    rotation.asDouble * 1.5 * (Math.PI),
                    gyroInputs.yaw.plus(driverOrientation),
                )
            } else {
                ChassisSpeeds(
                    forwards.asDouble * (maxSpeed into Units.MetersPerSecond),
                    strafe.asDouble * (maxSpeed into Units.MetersPerSecond),
                    rotation.asDouble * 1.5 * (Math.PI),
                )
            }

            applyChassisSpeeds(speeds)
        }
    }

    fun fakeNotePickup(): Command {
        return PrintCommand("Picking Up Note")
//        return this.run { applyChassisSpeeds(ChassisSpeeds(0.25, 0.0, 0.0)) }.withTimeout(2.0)
    }

    /**
     * Method for resetting the odometry of the robot
     *
     * @param pose New pose of the robot
     */
    fun resetOdometry(pose: Pose2d) {
        poseEstimator.resetPosition(gyroInputs.yaw, getModulePositions(), pose)
    }

    /**
     * Periodic method, runs every loop
     *
     * This method updates the gyro inputs, module inputs, and pose estimator
     */
    override fun periodic() {
        gyro.updateInputs(gyroInputs)
        Logger.processInputs("swerve/gyro", gyroInputs)
        modules.forEachIndexed { index, it ->
            it.updateInputs()
            Logger.processInputs("swerve/module[${index + 1}]", it.inputs)
        }
        poseEstimator.update(gyroInputs.yaw, getModulePositions())

        Logger.recordOutput("swerve/pose", Pose2d.struct, pose)
        modules.forEachIndexed { index, swerveModule ->
            measuredStates[index] = swerveModule.state
        }
        Logger.recordOutput("swerve/measuredState", SwerveModuleState.struct, *measuredStates)
        Logger.recordOutput("swerve/desiredState", SwerveModuleState.struct, *desiredStates)

        Logger.recordOutput(
            "vision/Estimator Camera Pose",
            Pose3d.struct,
            Pose3d(pose).transformBy(robotToCam),
        )

        if (!hasAppliedOffset || Robot.isDisabled) {
            DriverStation.getAlliance().ifPresent { alliance ->
                driverOrientation = when (alliance) {
                    DriverStation.Alliance.Red -> Rotation2d.fromDegrees(180.0)
                    DriverStation.Alliance.Blue -> Rotation2d()
                    else -> Rotation2d(0.0)
                }
                hasAppliedOffset = true
            }
        }
    }

    fun setVisionSTDDevs(xMeters: Double, yMeters: Double, rotRads: Double) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xMeters, yMeters, rotRads))
    }

    fun quasistaticSysID(direction: Direction): Command {
        return routineToUse.quasistatic(direction)
    }

    fun dynamicSysID(direction: Direction): Command {
        return routineToUse.dynamic(direction)
    }

    companion object {
        private val robotToCam: Transform3d = Transform3d(
            Translation3d(
                0.339736,
                Conversions.inchesToMeters(0.0),
                0.130902,
            ),
            Rotation3d(0.0, Conversions.degreesToRadians(-20.0), 0.0),
        )

        val maxSpeed = Units.FeetPerSecond.of(15.1)

        const val driveRatio: Double = 6.75
        const val turnRatio: Double = 150.0 / 7.0

        const val wheelRadiusInches: Double = 2.0

        val flConfig: ModuleIO.ModuleConstants = ModuleIO.ModuleConstants(
            1, 2, 3,
            false, true, false,
            Rotation2d.fromRotations(0.371),
            turnRatio, driveRatio,
            wheelRadiusInches
        )

        val frConfig: ModuleIO.ModuleConstants = ModuleIO.ModuleConstants(
            4, 5, 6,
            true, true, false,
            Rotation2d.fromRotations(0.39),
            turnRatio, driveRatio,
            wheelRadiusInches
        )

        val blConfig: ModuleIO.ModuleConstants = ModuleIO.ModuleConstants(
            7, 8, 9,
            false, true, false,
            Rotation2d.fromRotations(-0.386),
            turnRatio, driveRatio,
            wheelRadiusInches
        )


        val brConfig: ModuleIO.ModuleConstants = ModuleIO.ModuleConstants(
            10, 11, 12,
            true, true, false,
            Rotation2d.fromRotations(-0.247),
            turnRatio, driveRatio,
            wheelRadiusInches
        )
    }
}