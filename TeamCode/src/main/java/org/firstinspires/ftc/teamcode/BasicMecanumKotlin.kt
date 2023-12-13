package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import kotlin.math.*

@TeleOp(name="Basic Mecanum Kotlin", group="Jacob")
class BasicMecanumKotlin : OpMode() {
    companion object {
        // CONSTANTS
        const val YAW_ADJUST_SPEED = 0.005

        const val PIXEL_SCOOP_DEPLOYED = 0.4
        const val PIXEL_SCOOP_RETRACTED = 1.0

        const val MAX_ARM_EXTENDER_POSITION = 10500
        const val MIN_ARM_EXTENDER_POSITION = 1000
        const val ARM_EXTENDER_SPEED = 1.0

        const val MIN_ARM_ROTOR_POSITION = 0
        const val ARM_ROTOR_NEAR_CONTROLLER = 1000
        const val ARM_ROTOR_NEAR_FLOOR = 6000
        const val MAX_ARM_ROTOR_POSITION = 7400

        const val ARM_ROTOR_SPEED = 1.0
        const val ARM_ROTOR_THROTTLE = 0.5

        const val CLAW_JOINT_FOLDED = 0.825
        const val CLAW_JOINT_STRAIGHT = 0.175

        const val CLAW_OPEN = 0.0
        const val CLAW_CLOSED = 1.0

        const val AIRPLANE_LOADED = 0.0
        const val AIRPLANE_LAUNCHED = 1.0
    }

    // HELPERS
    private val runtime = ElapsedTime()
    private val driveBase = MecanumDrive()

    // HARDWARE MAPPINGS
    private lateinit var imu: IMU
    private lateinit var robotOrientation: YawPitchRollAngles
    private lateinit var rightFront: DcMotor
    private lateinit var leftFront: DcMotor
    private lateinit var leftRear: DcMotor
    private lateinit var rightRear: DcMotor
    private lateinit var armRotor: DcMotor
    private lateinit var armExtender: DcMotor

    private val compass = Compass()
    private lateinit var compassServo: CRServo
    private lateinit var compassEncoder: DcMotor

    private lateinit var clawJoint: Servo
    private lateinit var claw: Servo
    private lateinit var pixelScoop: Servo
    private lateinit var airplaneLauncher: Servo

    private lateinit var extensionLimit: TouchSensor

    // ROBOT STATE
    private var armRotorResetMode = false
    private var clawJointIsFolded = true

    override fun init() {
        telemetry.addData("Status", "Initializing")
        telemetry.update()

        // MAP HARDWARE
        imu             = hardwareMap.get(IMU::class.java, "imu")

        rightFront      = hardwareMap.get(DcMotor::class.java, "RightFront")
        leftFront       = hardwareMap.get(DcMotor::class.java, "LeftFront")
        leftRear        = hardwareMap.get(DcMotor::class.java, "LeftRear")
        rightRear       = hardwareMap.get(DcMotor::class.java, "RightRear")

        armRotor        = hardwareMap.get(DcMotor::class.java, "ArmRotor")
        armExtender     = hardwareMap.get(DcMotor::class.java, "ArmExtender")

        compassServo    = hardwareMap.get(CRServo::class.java, "CompassServo")
        compassEncoder  = hardwareMap.get(DcMotor::class.java, "CompassEncoder")

        clawJoint       = hardwareMap.get(Servo::class.java, "ClawJointServo")
        claw            = hardwareMap.get(Servo::class.java, "ClawServo")
        pixelScoop      = hardwareMap.get(Servo::class.java, "PixelScoopServo")
        airplaneLauncher = hardwareMap.get(Servo::class.java, "AirplaneLauncherServo")

        extensionLimit  = hardwareMap.get(TouchSensor::class.java, "ExtensionLimit")

        // CONFIGURE MOTORS
        rightFront.direction  = DcMotorSimple.Direction.FORWARD
        leftFront.direction   = DcMotorSimple.Direction.FORWARD
        leftRear.direction    = DcMotorSimple.Direction.FORWARD
        rightRear.direction   = DcMotorSimple.Direction.FORWARD

        armRotor.direction    = DcMotorSimple.Direction.FORWARD
        armExtender.direction = DcMotorSimple.Direction.FORWARD

        compassEncoder.direction = DcMotorSimple.Direction.FORWARD

        stopAndResetAllEncoders()

        clawJoint.position = CLAW_JOINT_FOLDED
        claw.position = CLAW_OPEN
        airplaneLauncher.position = AIRPLANE_LOADED

        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)))

        telemetry.addData("Status", "Initialized")
    }

    override fun start() {
        runtime.reset()
        setDefaultMotorModes()
        driveBase.resetYaw()
        imu.resetYaw()
    }

    override fun loop() {
        // Get controller inputs
        val forward = gamepad1.left_stick_y.toDouble()
        val right = gamepad1.left_stick_x.toDouble()
        val turn = gamepad2.left_stick_x.toDouble()
        val deltaYawTrim = YAW_ADJUST_SPEED * (gamepad1.right_trigger - gamepad2.left_trigger)

        var armExtensionRate = (gamepad2.right_trigger - gamepad2.left_trigger).toDouble()
        var armRotationRate = -gamepad2.right_stick_y.toDouble()

        if(gamepad2.back) {
            if(gamepad2.y) {
                armRotorResetMode = true
            }
            if(gamepad2.x) {
                armRotorResetMode = false
                armRotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                armRotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            }
        } else {
            // gamepad2.x folds, gamepad2.y unfolds
            if(clawJointIsFolded && gamepad2.y)
                clawJointIsFolded = false
            else if (!clawJointIsFolded && gamepad2.x)
                clawJointIsFolded = true
        }

        robotOrientation = imu.robotYawPitchRollAngles
        driveBase.yaw = robotOrientation.getYaw(AngleUnit.RADIANS)

        driveBase.yawTrim += deltaYawTrim

        if(gamepad1.b) {
            driveBase.resetYaw()
            imu.resetYaw()
        }

        compass.target = -driveBase.totalYaw
        compass.update(compassEncoder.currentPosition)
        compassServo.power = compass.idealPower

        driveBase.calculateDrivePower(right, forward, turn)
        rightFront.power    = driveBase.powerFrontRight
        leftFront.power     = driveBase.powerFrontLeft
        leftRear.power      = driveBase.powerRearLeft
        rightRear.power     = driveBase.powerRearRight

        // Rotate arm
        if(armRotor.currentPosition < ARM_ROTOR_NEAR_CONTROLLER && armRotationRate < 0) {
            armRotationRate *= ARM_ROTOR_THROTTLE
        }
        if(armRotor.currentPosition >= MAX_ARM_ROTOR_POSITION) {
            armRotationRate = min(armRotationRate, 0.0)
        }
        if(!armRotorResetMode && armRotor.currentPosition <= MIN_ARM_ROTOR_POSITION) {
            armRotationRate = max(armRotationRate, 0.0)
        }

        if(armRotorResetMode) {
            armRotationRate *= 0.5
        }

        armRotor.power = armRotationRate * ARM_ROTOR_SPEED

        // Extend arm
        if(extensionLimit.isPressed) {
            armExtensionRate = max(armExtensionRate, 0.0)

            armExtender.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            armExtender.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        if(armExtender.currentPosition >= MAX_ARM_EXTENDER_POSITION) {
            armExtensionRate = min(armExtensionRate, 0.0)
        }

        armExtender.power = armExtensionRate * ARM_EXTENDER_SPEED

        // Rotate clawJoint
        clawJoint.position = if (clawJointIsFolded) CLAW_JOINT_FOLDED else CLAW_JOINT_STRAIGHT

        // Deploy and retract pixelScoop
        if(gamepad2.a) {
            pixelScoop.position = PIXEL_SCOOP_DEPLOYED
        }
        if(gamepad2.b) {
            pixelScoop.position = PIXEL_SCOOP_RETRACTED
        }

        // Launch the plane
        if(gamepad1.dpad_up && gamepad1.a) {
            airplaneLauncher.position = AIRPLANE_LAUNCHED
        }

        // Show the elapsed game time and wheel power
        telemetry.addData("Status", "Run Time: $runtime")
        telemetry.addData("Rotor", armRotor.currentPosition)

        telemetry.addData("Yaw", "(%.1f)", Math.toDegrees(driveBase.totalYaw))

        telemetry.addData("Compass", "(%.2f)", compass.idealPower)
        telemetry.addData(" - angle", Math.toDegrees(compass.angle))

        telemetry.addData("Motors", "")
        telemetry.addData("", "(%.2f) (%.2f)", driveBase.powerFrontLeft, driveBase.powerFrontRight)
        telemetry.addData("", "(%.2f) (%.2f)", driveBase.powerRearLeft, driveBase.powerRearRight)

    }

    override fun stop() {
        stopAndResetAllEncoders()
    }

    private fun stopAndResetAllEncoders() {
        rightFront.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftFront.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftRear.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightRear.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        armRotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armExtender.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        compassEncoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        compassServo.power = 0.0
    }

    private fun setDefaultMotorModes() {
        rightFront.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftFront.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftRear.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightRear.mode = DcMotor.RunMode.RUN_USING_ENCODER

        armRotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armExtender.mode = DcMotor.RunMode.RUN_USING_ENCODER

        compassEncoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}