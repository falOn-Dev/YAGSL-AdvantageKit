package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ModuleIO {
    class ModuleIOInputs : LoggableInputs {
        var relativePosition: Double = 0.0
        var driveVelocity: Double = 0.0
        var drivePosition: Double = 0.0
        var absolutePosition: Double = 0.0
        var driveVoltage: Double = 0.0
        var angleVoltage: Double = 0.0

        override fun toLog(table: LogTable?) {
            table?.put("relativePosition", relativePosition)
            table?.put("driveVelocity", driveVelocity)
            table?.put("drivePosition", drivePosition)
            table?.put("absolutePosition", absolutePosition)
            table?.put("driveVoltage", driveVoltage)
            table?.put("angleVoltage", angleVoltage)
        }

        override fun fromLog(table: LogTable?) {
            table?.get("relativePosition")?.let { relativePosition = it.double }
            table?.get("driveVelocity")?.let { driveVelocity = it.double }
            table?.get("drivePosition")?.let { drivePosition = it.double }
            table?.get("absolutePosition")?.let { absolutePosition = it.double }
            table?.get("driveVoltage")?.let { driveVoltage = it.double }
            table?.get("angleVoltage")?.let { angleVoltage = it.double }
        }

    }

    fun updateInputs(inputs: ModuleIOInputs) {}

}