package frc.robot

import SwerveSubsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.Autos
import frc.robot.commands.swerve.CornerSpinCommand
import frc.robot.commands.swervedrive.drivebase.TeleopDrive
import frc.robot.commands.vision.TrackTargetCommand
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.util.SingletonXboxController

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

    val controller = SingletonXboxController

    val limelight = LimelightSubsystem
    val drivebase = SwerveSubsystem

    val trackTarget = TrackTargetCommand()

    // TODO: This is kinda weird but inverting (and the drive encoders) makes it display properly
    //     No, uninverting both doesn't fix it :(
    val teleopDrive: TeleopDrive =
        TeleopDrive(
            { -controller.leftY },
            { -controller.leftX },
            { -controller.rightX },
            { controller.hid.leftBumper },
            { controller.hid.rightBumper },
        )

    val cornerSpin: CornerSpinCommand =
        CornerSpinCommand(
            { -controller.rightX },
            { controller.hid.leftBumper },
            { controller.hid.rightBumper },
        )

    init {
        configureBindings()

        drivebase.defaultCommand = teleopDrive
        // Reference the Autos object so that it is initialized, placing the chooser on the dashboard
        Autos
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
        controller.b().onTrue(drivebase.runOnce { drivebase.zeroGyro() })

        controller.a().toggleOnTrue(trackTarget)

        controller.x().toggleOnTrue(cornerSpin)
    }
}
