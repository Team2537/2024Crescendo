package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.Autos
import frc.robot.commands.swerve.*
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.util.SingletonXboxController
import lib.profiles.DriverProfile
import lib.zones.Zones

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {
    // Testing pre-commit

    private val controller = SingletonXboxController // TODO: refactor to use ProfileController


//    val trackTarget = TrackTargetCommand()

    // TODO: This is kinda weird but inverting (and the drive encoders) makes it display properly
    //     No, uninverting both doesn't fix it :(
    val teleopDrive: TeleopDriveCommand =
        TeleopDriveCommand(
            { -controller.leftY },
            { -controller.leftX },
            { -controller.rightX },
            { !controller.hid.leftBumper },
            { controller.hid.rightBumper },
        )

    val cornerSpin: CornerSpinCommand =
        CornerSpinCommand(
            { -controller.rightX },
            { controller.hid.leftBumper },
            { controller.hid.rightBumper },
        )

    val absoluteDrive: AbsoluteDriveCommand = AbsoluteDriveCommand(
        { -controller.leftY },
        { -controller.leftX },
        { -controller.rightX },
        { -controller.rightY }
    )

    val faceClosestFieldElementCommand: AbsoluteDriveElementListRelativeCommand = AbsoluteDriveElementListRelativeCommand(
        listOf(Constants.FieldConstants.BLUE_AMP,
            Constants.FieldConstants.BLUE_SOURCE,
            Constants.FieldConstants.BLUE_SPEAKER
        ),
        { -controller.leftY },
        { -controller.leftX }
    )

    val faceSpeakerCommand: AbsoluteDriveElementRelativeCommand = AbsoluteDriveElementRelativeCommand(
        Constants.FieldConstants.BLUE_SPEAKER,
        { -controller.leftY },
        { -controller.leftX }
    )

    val faceAmpCommand: AbsoluteDriveElementRelativeCommand = AbsoluteDriveElementRelativeCommand(
        Constants.FieldConstants.BLUE_AMP,
        { -controller.leftY },
        { -controller.leftX }
    )

    val faceSourceCommand: AbsoluteDriveElementRelativeCommand = AbsoluteDriveElementRelativeCommand(
        Constants.FieldConstants.BLUE_SOURCE,
        { -controller.leftY },
        { -controller.leftX }
    )

    val faceSpeakerTrigger: Trigger = Trigger { Zones[SwerveSubsystem.getPose()] == Zones["speaker"] && controller.hid.leftTriggerAxis > 0.75 }
    val faceAmpTrigger: Trigger = Trigger { Zones[SwerveSubsystem.getPose()] == Zones["amp"] && controller.hid.leftTriggerAxis > 0.75 }
    val faceSourceTrigger: Trigger = Trigger { Zones[SwerveSubsystem.getPose()] == Zones["source"] && controller.hid.leftTriggerAxis > 0.75 }

    var speakerPub: StructPublisher<Pose2d> = NetworkTableInstance.getDefault().
            getStructTopic("FieldConstants/BLUE_SPEAKER", Pose2d.struct).publish()
    var ampPub: StructPublisher<Pose2d> = NetworkTableInstance.getDefault().
            getStructTopic("FieldConstants/BLUE_AMP", Pose2d.struct).publish()
    var sourcePub: StructPublisher<Pose2d> = NetworkTableInstance.getDefault().
            getStructTopic("FieldConstants/BLUE_SOURCE", Pose2d.struct).publish()


    init {
        // TODO: comment stuff in this function cause I'm lazy (:
        initializeObjects()

        configureBindings()
        SwerveSubsystem.defaultCommand = teleopDrive
    }

    /**
     * Use to eager initialize objects
     */
    private fun initializeObjects() {
        Autos
        SwerveSubsystem
//        LimelightSubsystem
        DriverProfile
    }

    public fun periodic() {
        speakerPub.set(Constants.FieldConstants.BLUE_SPEAKER)
        ampPub.set(Constants.FieldConstants.BLUE_AMP)
        sourcePub.set(Constants.FieldConstants.BLUE_SOURCE)

    }

    // Replace with CommandPS4Controller or CommandJoystick if needed

    /**
     * Use this method to define your `trigger->command` mappings. Triggers can be created via the
     * [Trigger] constructor that takes a [BooleanSupplier][java.util.function.BooleanSupplier]
     * with an arbitrary predicate, or via the named factories in [GenericHID][edu.wpi.first.wpilibj2.command.button.CommandGenericHID]
     * subclasses such for [Xbox][CommandXboxController]/[PS4][edu.wpi.first.wpilibj2.command.button.CommandPS4Controller]
     * controllers or [Flight joysticks][edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() {
        controller.a().onTrue(InstantCommand(SwerveSubsystem::zeroGyro))
        controller.y().toggleOnTrue(absoluteDrive)
        controller.x().toggleOnTrue(faceClosestFieldElementCommand)

        faceSpeakerTrigger.whileTrue(faceSpeakerCommand)
        faceAmpTrigger.whileTrue(faceAmpCommand)
        faceSourceTrigger.whileTrue(faceSourceCommand)
    }
}
