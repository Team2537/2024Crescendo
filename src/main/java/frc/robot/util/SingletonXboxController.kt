package frc.robot.util

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.Constants


object SingletonXboxController : CommandXboxController(Constants.IOConstants.DRIVER_CONTROLLER_PORT){}