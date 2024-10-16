package frc.robot.subsystems.swerve

import choreo.Choreo
import choreo.Choreo.TrajectoryLogger
import choreo.auto.AutoFactory
import choreo.trajectory.SwerveSample
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
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
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
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
import lib.LoggedTunableNumber
import lib.math.units.into
import org.littletonrobotics.junction.Logger
import java.util.function.BiConsumer
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import kotlin.jvm.optionals.getOrDefault
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
        Translation2d(moduleOffset, moduleOffset),
        Translation2d(moduleOffset, -moduleOffset),
        Translation2d(-moduleOffset, moduleOffset),
        Translation2d(-moduleOffset, -moduleOffset),
    )

    private val driveKp = LoggedTunableNumber(
        "swerve/modules/driveKp",
        if (Constants.RobotConstants.mode == Constants.RobotConstants.Mode.REAL) 5.0 else 0.0
    )

    private val driveKi = LoggedTunableNumber(
        "swerve/modules/driveKi",
        if (Constants.RobotConstants.mode == Constants.RobotConstants.Mode.REAL) 0.0 else 0.0
    )

    private val driveKd = LoggedTunableNumber(
        "swerve/modules/driveKd",
        if (Constants.RobotConstants.mode == Constants.RobotConstants.Mode.REAL) 0.1 else 0.0
    )

    private val turnKp = LoggedTunableNumber(
        "swerve/modules/turnKp",
        if (Constants.RobotConstants.mode == Constants.RobotConstants.Mode.REAL) 5.0 else 10.0
    )

    private val turnKi = LoggedTunableNumber(
        "swerve/modules/turnKi",
        if (Constants.RobotConstants.mode == Constants.RobotConstants.Mode.REAL) 0.0 else 0.0
    )

    private val turnKd = LoggedTunableNumber(
        "swerve/modules/turnKd",
        if (Constants.RobotConstants.mode == Constants.RobotConstants.Mode.REAL) 0.0 else 0.0
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

    private val slowModeScalar = 0.3

    private val xControlP = LoggedTunableNumber("swerve/choreo/xControlP", 4.0)
    private val xControlI = LoggedTunableNumber("swerve/choreo/xControlI", 0.1)
    private val xControlD = LoggedTunableNumber("swerve/choreo/xControlD", 1.5)

    private val yControlP = LoggedTunableNumber("swerve/choreo/yControlP", 4.0)
    private val yControlI = LoggedTunableNumber("swerve/choreo/yControlI", 0.1)
    private val yControlD = LoggedTunableNumber("swerve/choreo/yControlD", 1.5)

    private val rotControlP = LoggedTunableNumber("swerve/choreo/rotControlP", 0.0)
    private val rotControlI = LoggedTunableNumber("swerve/choreo/rotControlI", 0.0)
    private val rotControlD = LoggedTunableNumber("swerve/choreo/rotControlD", 0.0)

    private val xControl = PIDController(xControlP.get(), xControlI.get(), xControlD.get())
    private val yControl = PIDController(yControlP.get(), yControlI.get(), yControlD.get())
    private val rotControl = PIDController(rotControlP.get(), rotControlI.get(), rotControlD.get())

    private val controller: BiConsumer<Pose2d, SwerveSample> =
        BiConsumer { currPose: Pose2d, sample: SwerveSample ->
            val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xControl.calculate(currPose.translation.x, sample.x) + sample.vx,
                yControl.calculate(currPose.translation.y, sample.y) + sample.vy,
                rotControl.calculate(currPose.rotation.radians, sample.heading) + sample.omega,
                pose.rotation
            )

            applyChassisSpeeds(speeds)
        }

    private val trajLogger: TrajectoryLogger<SwerveSample> =
        TrajectoryLogger { sample, isStart ->
            if (!isStart) {
                Logger.recordOutput("swerve/AutoTraj", *emptyArray<Pose2d>())
                return@TrajectoryLogger
            }


            if (DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
                Logger.recordOutput("swerve/AutoTraj", *sample.flipped().poses)
            } else {
                Logger.recordOutput("swerve/AutoTraj", *sample.poses)
            }
        }

    val factory: AutoFactory = Choreo.createAutoFactory(
        this,
        ::pose,
        controller,
        { DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red },
        AutoFactory.AutoBindings(),
        trajLogger
    )

    init {
        modules.forEach {
            it.setDrivePID(driveKp.get(), driveKi.get(), driveKd.get())
            it.setTurnPID(turnKp.get(), turnKi.get(), turnKd.get())
        }
    }

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
        gyro.setYaw(Rotation2d())
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
        isSlowMode: BooleanSupplier
    ): Command? {
        return this.run {
            var vForwards = forwards.asDouble
            var vStrafe = strafe.asDouble
            var vRotation = rotation.asDouble

            if (isSlowMode.asBoolean) {
                vForwards *= slowModeScalar
                vStrafe *= slowModeScalar
                vRotation *= slowModeScalar
            }

            val speeds = if (isFieldOriented.asBoolean) {
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    (vForwards * (maxSpeed into Units.MetersPerSecond)),
                    (vStrafe * (maxSpeed into Units.MetersPerSecond)),
                    (vRotation * 1.5 * (Math.PI)),
                    gyroInputs.yaw.plus(driverOrientation),
                )
            } else {
                ChassisSpeeds(
                    (vForwards * (maxSpeed into Units.MetersPerSecond)),
                    (vStrafe * (maxSpeed into Units.MetersPerSecond)),
                    (vRotation * 1.5 * (Math.PI)),
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

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { pid -> xControl.setPID(pid[0], pid[1], pid[2]) },
            xControlP, xControlI, xControlD
        )

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { pid -> yControl.setPID(pid[0], pid[1], pid[2]) },
            yControlP, yControlI, yControlD
        )

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { pid -> rotControl.setPID(pid[0], pid[1], pid[2]) },
            rotControlP, rotControlI, rotControlD
        )

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { pid -> modules.forEach { it.setDrivePID(pid[0], pid[1], pid[2]) }; println("Drive PID: $pid") },
            driveKp, driveKi, driveKd
        )

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { pid -> modules.forEach { it.setTurnPID(pid[0], pid[1], pid[2]) }; println("Turn PID: $pid") },
            turnKp, turnKi, turnKd
        )

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

    fun setVisionSTDDevs(x: Measure<Distance>, y: Measure<Distance>, rot: Measure<Angle>) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(x into Meters, y into Meters, rot into Radians))
    }

    fun quasistaticSysID(direction: Direction): Command {
        return routineToUse.quasistatic(direction)
    }

    fun dynamicSysID(direction: Direction): Command {
        return routineToUse.dynamic(direction)
    }

    fun stop() {
        applyChassisSpeeds(ChassisSpeeds())
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

        val wheelRadius = Units.Inches.of(2.0)

        val moduleOffset = inchesToMeters(9.7859)

        val flConfig: ModuleIO.ModuleConstants = ModuleIO.ModuleConstants(
            1, 2, 3,
            false, true, false,
            Rotation2d.fromRotations(0.371),
            turnRatio, driveRatio,
            wheelRadius
        )

        val frConfig: ModuleIO.ModuleConstants = ModuleIO.ModuleConstants(
            4, 5, 6,
            true, true, false,
            Rotation2d.fromRotations(0.39),
            turnRatio, driveRatio,
            wheelRadius
        )

        val blConfig: ModuleIO.ModuleConstants = ModuleIO.ModuleConstants(
            7, 8, 9,
            false, true, false,
            Rotation2d.fromRotations(-0.386),
            turnRatio, driveRatio,
            wheelRadius
        )


        val brConfig: ModuleIO.ModuleConstants = ModuleIO.ModuleConstants(
            10, 11, 12,
            true, true, false,
            Rotation2d.fromRotations(-0.247),
            turnRatio, driveRatio,
            wheelRadius
        )
    }
}