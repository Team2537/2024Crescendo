package frc.robot

import LauncherSubsystem
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.Autos
import frc.robot.commands.climb.ManualClimbCommand
import frc.robot.commands.intake.ManualIntakeCommand
import frc.robot.commands.intake.TestTransfer
import frc.robot.commands.intake.ToggleIntakeCommand
import frc.robot.commands.launcher.*
import frc.robot.commands.pivot.*
import frc.robot.subsystems.*
//import frc.robot.subsystems.SwerveSubsystem
import frc.robot.util.SingletonXboxController
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

    val swerve: SwerveSubsystem = SwerveSubsystem()

    val launcherIsUsed: Trigger = Trigger() { CommandScheduler.getInstance().requiring(LauncherSubsystem) == null }


    val manualIntake: ManualIntakeCommand = ManualIntakeCommand(
        { -controller.rightTriggerAxis },
        { controller.leftTriggerAxis }
    )

//    val trackSpeakerCommand = ParallelCommandGroup(
//        RotateTowardsTargetCommand(LimelightSubsystem.odometryLimelight),
//        QuickPivotCommand(0.0, false, true)
//    )

    val intakePivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.INTAKE_POSITION, false, false)
    val subwooferPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.SUBWOOFER_POSITION, false, false)
    val ampPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.AMP_POSITION, false, false)
    val podiumPivot: QuickPivotCommand = QuickPivotCommand(Constants.PivotConstants.MID_POSITION, false, false)
    val autoAim: QuickPivotCommand = QuickPivotCommand(0.0, false, true)

    val manualPivot: ManualPivotCommand = ManualPivotCommand() { controller.rightY }

    val manualClimb: ManualClimbCommand = ManualClimbCommand() { controller.rightY }

    val launchCommand: LaunchCommand = LaunchCommand(
        {1.0},
        { controller.leftTrigger(0.75).asBoolean },
        { PivotSubsystem.getRelativePosition() },
        { controller.button(Constants.OperatorConstants.START_BUTTON).asBoolean }
    )

    val teleopDrive: Command = swerve.driveCommand(
        { -controller.leftY },
        { -controller.leftX },
        { -controller.rightX }
    )





    init {
        // TODO: comment stuff in this function cause I'm lazy (:
        initializeObjects()
        configureBindings()
        swerve.defaultCommand = teleopDrive

        Shuffleboard.getTab("Scheduler").add("Scheduler", CommandScheduler.getInstance())

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
        ClimbSubsystem
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
        controller.leftBumper().toggleOnTrue(
            ParallelDeadlineGroup(
                IntakeNoteCommand(),
                ToggleIntakeCommand().alongWith(
                    QuickPivotCommand(Constants.PivotConstants.INTAKE_POSITION, false, false)
                )
            )
        )

//        controller.rightBumper().onTrue(
//            Commands.runOnce({
//                if (teleopDrive.isScheduled) {
//                    teleopDrive.cancel()
//                    trackSpeakerCommand.schedule()
//                } else {
//                    trackSpeakerCommand.cancel()
//                    teleopDrive.schedule()
//                }
//            })
//        )

        controller.leftStick().onTrue(InstantCommand(swerve::zeroGyro))
        controller.povUp().onTrue(ampPivot)
        controller.povRight().onTrue(intakePivot)
        controller.povDown().onTrue(subwooferPivot)
        controller.povLeft().onTrue(
            autoAim
        ) // TODO: Implement auto-aiming

        controller.y().onTrue(HomePivotCommand()) // TODO: Implement homing launcher
        controller.b().toggleOnTrue(manualPivot)
        controller.x().onTrue(manualClimb)
//        controller.rightBumper().toggleOnTrue(manualClimb)
        controller.rightStick().onTrue(InstantCommand(swerve::toggleFieldOriented))
        controller.button(Constants.OperatorConstants.BACK_BUTTON)
            .toggleOnTrue(TestTransfer()) // TODO: Implement Toggle
//        controller.button(Constants.OperatorConstants.START_BUTTON)
//            .onTrue(Commands.runOnce({
//                SwerveSubsystem.resetOdometry(Constants.FIELD_LOCATIONS.SUBWOOFER_POSE)
//            })) // TODO: Implement Intake Command Override


        LauncherSubsystem.noteTrigger.and(controller.a()).onTrue(launchCommand) // TODO: Implement Priming



    }


}
