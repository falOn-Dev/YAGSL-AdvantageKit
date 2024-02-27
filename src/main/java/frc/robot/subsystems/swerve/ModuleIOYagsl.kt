package frc.robot.subsystems.swerve

import edu.wpi.first.math.kinematics.SwerveModuleState
import swervelib.SwerveModule
import swervelib.motors.SwerveMotor

class ModuleIOYagsl(module: SwerveModule) : ModuleIO, SwerveModule(module.moduleNumber, module.configuration, module.feedforward) {

    private val swerveModule: SwerveModule = module
    private val driveMotor: SwerveMotor = swerveModule.driveMotor
    private val angleMotor: SwerveMotor = swerveModule.angleMotor


    override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
        inputs.drivePosition = driveMotor.position
        inputs.driveVelocity = driveMotor.velocity
        inputs.relativePosition = angleMotor.position
        inputs.absolutePosition= swerveModule.absolutePosition
        inputs.driveVoltage = driveMotor.voltage * driveMotor.appliedOutput
        inputs.angleVoltage = angleMotor.voltage * angleMotor.appliedOutput
    }

}