package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.swerve.ModuleIO
import frc.robot.subsystems.swerve.ModuleIOYagsl
import frc.robot.subsystems.swerve.SwerveDriveAK
import org.littletonrobotics.junction.Logger
import swervelib.SwerveDrive
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import java.io.File
import java.nio.file.Path

/**
 * The subsystem that controls the swerve drive.
 *
 * @constructor Creates a SwerveSubsystem from JSON Configuration Files.
 * @author Falon Clark
 * @since 1/15/2024
 */
object SwerveSubsystem : SubsystemBase() {
    private val swerveDrive: SwerveDrive

    /** @suppress */
    var maximumSpeed: Double = Units.feetToMeters(9.0)
    var tab: ShuffleboardTab = Shuffleboard.getTab("Testing")
    var swerveStates: StructArrayPublisher<SwerveModuleState> = NetworkTableInstance.getDefault().
    getStructArrayTopic("SwerveStates/swerveStates", SwerveModuleState.struct).publish()

    var moduleIOList: Array<ModuleIO?> = Array(4) { null }

    var inputsArray: Array<ModuleIO.ModuleIOInputs> = arrayOf(
        ModuleIO.ModuleIOInputs(),
        ModuleIO.ModuleIOInputs(),
        ModuleIO.ModuleIOInputs(),
        ModuleIO.ModuleIOInputs()
    )


    init {

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH

        try {
            swerveDrive = SwerveParser(
                File(Filesystem.getDeployDirectory(),
                    "/swerve/")
            ).createSwerveDrive(maximumSpeed)
        } catch (e: Exception) {
            e.printStackTrace()
            throw RuntimeException("Error creating swerve drive", e)
            throw RuntimeException("Error creating swerve drive", e)
        }

        swerveDrive.setHeadingCorrection(false)

        setMotorBrake(true)

        tab.addDouble("Heading") { getHeading().degrees }

        for(i in 0..3) {
            moduleIOList[i] = ModuleIOYagsl(swerveDrive.modules[i])
        }

    }



    /**
     * Simple drive method that translates and rotates the robot.
     * @param translation The desired X and Y velocity of the robot.
     * @param rotation The desired rotational velocity of the robot.
     * @param fieldOriented Whether the robot's motion should be field oriented or robot oriented.
     */
    fun drive(
        translation: Translation2d,
        rotation: Double,
        fieldOriented: Boolean,
    ) {
        swerveDrive.drive(translation, rotation, fieldOriented, false)
    }

    /**
     * Advanced drive method that translates and rotates the robot, with a custom center of rotation.
     * @param translation The desired X and Y velocity of the robot.
     * @param rotation The desired rotational velocity of the robot.
     * @param fieldOriented Whether the robot's motion should be field oriented or robot oriented.
     * @param centerOfRotation The center of rotation of the robot.
     */
    fun drive(
        translation: Translation2d,
        rotation: Double,
        fieldOriented: Boolean,
        centerOfRotation: Translation2d,
    ) {
        swerveDrive.drive(translation, rotation, fieldOriented, false, centerOfRotation)
    }

    /**
     * Simple drive method that uses ChassisSpeeds to control the robot.
     * @param velocity The desired ChassisSpeeds of the robot
     */
    fun drive(velocity: ChassisSpeeds) {
        swerveDrive.drive(velocity)
    }

    fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds)
    }

    /**
     * Method to get the Kinematics object of the swerve drive.
     */
    fun getKinematics() = swerveDrive.kinematics

    /**
     * Method to reset the odometry of the robot to a desired pose.
     * @param initialHolonomicPose The desired pose to reset the odometry to.
     */
    fun resetOdometry(initialHolonomicPose: Pose2d) {
        swerveDrive.resetOdometry(initialHolonomicPose)
    }

    /**
     * Method to get the current pose of the robot.
     * @return The current pose of the robot.
     */
    fun getPose() = swerveDrive.pose

    /**
     * Method to display a desired trajectory to a field2d object.
     */
    fun postTrajectory(trajectory: Trajectory) {
        swerveDrive.postTrajectory(trajectory)
    }

    /**
     * Method to zero the gyro.
     */
    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    /**
     * Method to toggle the motor's brakes.
     * @param brake Whether to set the motor's brakes to true or false.
     */
    fun setMotorBrake(brake: Boolean) {
        swerveDrive.setMotorIdleMode(brake)
    }

    /**
     * Method to get the current heading of the robot.
     * @return The current heading of the robot.
     */
    fun getHeading() = swerveDrive.yaw

    /**
     * Method to generate a ChassisSpeeds object from a desired X, Y, and Rotational velocity.
     * @param vForward The desired forward velocity of the robot.
     * @param vSide The desired side velocity of the robot.
     * @param angle The desired rotational velocity of the robot.
     * @return The generated ChassisSpeeds object.
     */
    fun getTargetSpeeds(
        vForward: Double,
        vSide: Double,
        angle: Rotation2d,
    ): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(vForward, vSide, angle.radians, getHeading().radians, maximumSpeed)
    }

    fun getTargetSpeeds(
        vForward: Double,
        vSide: Double,
        headingX: Double,
        headingY: Double
    ): ChassisSpeeds {
        return swerveDrive.swerveController.getTargetSpeeds(vForward, vSide, headingX, headingY, getHeading().radians, maximumSpeed)
    }

    /**
     * Method to get the current field oriented velocity of the robot.
     * @return The current field oriented velocity of the robot.
     */
    fun getFieldVelocity(): ChassisSpeeds? {
        return swerveDrive.fieldVelocity
    }

    /**
     * Method to get the current robot oriented velocity of the robot.
     * @return The current robot oriented velocity of the robot.
     */
    fun getRobotVelocity(): ChassisSpeeds? {
        return swerveDrive.robotVelocity
    }

    /**
     * Method to get the SwerveController object of the swerve drive.
     * @return The SwerveController object of the swerve drive.
     */
    fun getSwerveController() = swerveDrive.swerveController

    /**
     * Method to get the SwerveDriveConfiguration object of the swerve drive.
     * @return The SwerveDriveConfiguration object of the swerve drive.
     */
    fun getSwerveDriveConfiguration() = swerveDrive.swerveDriveConfiguration

    /**
     * Method to toggle the lock position of the swerve drive to prevent motion.
     */
    fun lock() {
        swerveDrive.lockPose()
    }

    /**
     * Method to get the current pitch of the robot.
     */
    fun getPitch() = swerveDrive.pitch

    fun addVisionMeasurement(measurement: Pose2d, timestamp: Double) {
        swerveDrive.addVisionMeasurement(measurement, timestamp)
    }

    override fun periodic() {
        swerveStates.set(swerveDrive.states)
        for(i in 0..3) {
            moduleIOList[i]?.updateInputs(inputsArray[i])
            Logger.processInputs("swerve/Module[${i + 1}]", inputsArray[i])
        }

    }
}
