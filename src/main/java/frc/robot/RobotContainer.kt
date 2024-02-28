package frc.robot

import LauncherSubsystem
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.Autos
import frc.robot.commands.intake.ManualIntakeCommand
import frc.robot.commands.intake.ToggleIntakeCommand
import frc.robot.commands.launcher.*
import frc.robot.commands.launcher.pivot.ManualPivotCommand
import frc.robot.commands.launcher.pivot.QuickPivotCommand
import frc.robot.commands.swerve.AbsoluteDriveCommand
import frc.robot.commands.swerve.CornerSpinCommand
import frc.robot.commands.swerve.TeleopDriveCommand
import frc.robot.commands.vision.TrackTargetCommand
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.subsystems.PivotSubsystem
//import frc.robot.subsystems.SwerveSubsystem
import frc.robot.util.SingletonXboxController
import lib.math.units.rotations
import lib.profiles.DriverProfile

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
            { controller.hid.leftBumper },
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

    val manualIntake: ManualIntakeCommand = ManualIntakeCommand(
        { -controller.rightTriggerAxis },
        { controller.leftTriggerAxis }
    )

    val manualPivot: ManualPivotCommand = ManualPivotCommand() { controller.rightY }


    val ampQuickPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.AMP_POSITION)
    val speakerQuickPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION)
    val intakeQuickPivot: QuickPivotCommand = QuickPivotCommand(0.21)



    init {
        // TODO: comment stuff in this function cause I'm lazy (:
        initializeObjects()

        configureBindings()

//        SwerveSubsystem.defaultCommand = null
        IntakeSubsystem.defaultCommand = manualIntake
    }

    /**
     * Use to eager initialize objects
     */
    private fun initializeObjects() {
        Autos
//        SwerveSubsystem
        Autos
//        LimelightSubsystem
        DriverProfile
        PivotSubsystem
        LauncherSubsystem
        IntakeSubsystem
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
//        controller.a().onTrue(InstantCommand(SwerveSubsystem::zeroGyro))
//        controller.y().toggleOnTrue(absoluteDrive)
        controller.b().toggleOnTrue(manualPivot)
        controller.pov(90).onTrue(ampQuickPivot)
        controller.pov(270).onTrue(speakerQuickPivot)
        controller.pov(180).onTrue(intakeQuickPivot)
        controller.leftBumper().onTrue(InstantCommand(PivotSubsystem::zeroEncoder))
//        intakeTrigger.whileTrue(ToggleIntakeCommand())
        stateBindings()
    }


    private fun stateBindings(){
        LauncherSubsystem.triggerFactory(LauncherSubsystem.State.EMPTY)
        LauncherSubsystem.triggerFactory(LauncherSubsystem.State.STORED).whileTrue(IntakeCommand())
        LauncherSubsystem.triggerFactory(LauncherSubsystem.State.PRIMED).whileTrue(PrimeLauncherCommand())
        LauncherSubsystem.triggerFactory(LauncherSubsystem.State.AT_SPEED)
            .and(controller.leftTrigger()).onTrue(ReadyFireCommand())
        LauncherSubsystem.triggerFactory(LauncherSubsystem.State.FIRING).whileTrue(FireCommand())
    }

    private fun addNamedCommands() {
        NamedCommands.registerCommand("Intake", PrintCommand("Intake"))
        NamedCommands.registerCommand("Launch", PrintCommand("Launch"))
    }
}
