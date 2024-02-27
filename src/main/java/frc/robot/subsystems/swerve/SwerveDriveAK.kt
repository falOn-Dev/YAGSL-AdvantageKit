package frc.robot.subsystems.swerve

import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import swervelib.SwerveDrive

class SwerveDriveAK(swerveDrive: SwerveDrive) : SwerveDrive(
    swerveDrive.swerveDriveConfiguration,
    swerveDrive.swerveController.config,
    Units.feetToMeters(12.0)) {

    val IOModules: Array<ModuleIO> = arrayOf(
        ModuleIOYagsl(swerveDrive.modules[0]),
        ModuleIOYagsl(swerveDrive.modules[1]),
        ModuleIOYagsl(swerveDrive.modules[2]),
        ModuleIOYagsl(swerveDrive.modules[3])
    )

    val flModuleInputs: ModuleIO.ModuleIOInputs = ModuleIO.ModuleIOInputs()
    val frModuleInputs: ModuleIO.ModuleIOInputs = ModuleIO.ModuleIOInputs()
    val blModuleInputs: ModuleIO.ModuleIOInputs = ModuleIO.ModuleIOInputs()
    val brModuleInputs: ModuleIO.ModuleIOInputs = ModuleIO.ModuleIOInputs()

    fun updateInputs() {
        IOModules[0].updateInputs(flModuleInputs)
        IOModules[1].updateInputs(frModuleInputs)
        IOModules[2].updateInputs(blModuleInputs)
        IOModules[3].updateInputs(brModuleInputs)
    }

    fun processInputs() {
        Logger.processInputs("swerve/Module[1]", flModuleInputs)
        Logger.processInputs("swerve/Module[2]", frModuleInputs)
        Logger.processInputs("swerve/Module[3]", blModuleInputs)
        Logger.processInputs("swerve/Module[4]", brModuleInputs)
    }
}