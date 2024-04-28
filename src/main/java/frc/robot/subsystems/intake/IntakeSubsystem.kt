package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

object IntakeSubsystem : SubsystemBase() {
    val io: IntakeIO
    val input: IntakeIO.IntakeIOInputs = IntakeIO.IntakeIOInputs()

    init {
        io = IntakeIONeos()
    }

    fun intakeNote(): Command? {
        return this.runEnd({
            io.setIntakePower(-1.0)
            io.setTransferPower(1.0)
        }, {
            io.stopIntake()
            io.stopTransfer()
        })
    }

    fun feedNote(): Command? {
        return this.runEnd({
            io.setTransferPower(1.0)
        }, {
            io.stopTransfer()
        })
    }

    override fun periodic() {
        io.updateInputs(input)
        Logger.processInputs("Intake", input)
    }
}