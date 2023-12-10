package frc.robot.subsystems

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.SubsystemBase;

object LimelightSubsystem : SubsystemBase() {

    private var tx: NetworkTableEntry
    private var ty: NetworkTableEntry
    private var ta: NetworkTableEntry
    private var tv: NetworkTableEntry
    private var ts: NetworkTableEntry

    private var table: NetworkTable

    var xOffset: Double = 0.0
    var yOffset: Double = 0.0
    var area: Double = 0.0
    var skew: Double = 0.0
    var targetVisible: Boolean = false

    private var visionTab: ShuffleboardTab

    init {
        table = NetworkTableInstance.getDefault().getTable("limelight")

        tx = table.getEntry("tx")
        ty = table.getEntry("ty")
        ta = table.getEntry("ta")
        tv = table.getEntry("tv")
        ts = table.getEntry("ts")

        visionTab = Shuffleboard.getTab("Vision")

        visionTab.addDouble("X Offset") { xOffset }
        visionTab.addDouble("Y Offset") { yOffset }
        visionTab.addDouble("Area") { area }
        visionTab.addDouble("Skew") { skew }
        visionTab.addBoolean("Target Visible") { targetVisible }



    }

    override fun periodic() {
        xOffset = tx.getDouble(0.0)
        yOffset = ty.getDouble(0.0)
        area = ta.getDouble(0.0)
        skew = ts.getDouble(0.0)
        targetVisible = tv.getDouble(0.0) > 0
    }
}