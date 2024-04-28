package frc.robot.subsystems

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import lib.vision.Limelight

/**
 * The subsystem that controls the limelight.
 * This subsystem is heavily underdeveloped.
 * And may be removed in the future with the replacement of PhotonVision
 */
object LimelightSubsystem : SubsystemBase() {
    val odometryLimelight: Limelight = Limelight("limelight-odom")
    //val intakeLimelight: Limelight = Limelight("limelight-intake")

    init {
        odometryLimelight.setTargetTag(7)
        //intakeLimelight.setLEDs(false)
        odometryLimelight.setLEDs(true)

        if (DriverStation.getAlliance().isPresent) {
            println("Limelight target set, alliance: ${DriverStation.getAlliance()}")
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                odometryLimelight.setTargetTag(4)
            } else {
                odometryLimelight.setTargetTag(7)
            }
        }
    }
}
