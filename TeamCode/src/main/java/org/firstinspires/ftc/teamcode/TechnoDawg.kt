package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

object TechnoDawg {
    /***********************************************************************************************
     * CONSTANTS ***********************************************************************************
     ***********************************************************************************************/
    const val YAW_ADJUST_SPEED = 0.02

    private const val ARM_EXTENDER_SPEED = 1.0

    private const val ARM_ROTOR_SPEED = 1.0

    enum class RotorPosition (override val position: Int) : DcMotorPosition {
        STOWED(0),
        UP(3500),
        BACKBOARD(5000),
        FORWARD(7300),
        GRAB(7800)
    }

    enum class ExtenderPosition (override val position: Int) : DcMotorPosition {
        MIN(0),
        GRAB(2200),
        MAX(10500)
    }

    enum class JointPosition (override val position: Double) : ServoPosition {
        FOLDED(0.835),
        STRAIGHT(0.175)
    }

    enum class ScoopPosition (override val position: Double) : ServoPosition {
        UP(1.0),
        DOWN(0.4)
    }

    enum class ClawPosition (override val position: Double) : ServoPosition {
        OPEN (1.0),
        CLOSED (0.0)
    }

    enum class LauncherPosition (override val position: Double) : ServoPosition {
        LOADED (0.0),
        LAUNCHED (1.0)
    }

    /***********************************************************************************************
     * ROBOT STATE *********************************************************************************
     ***********************************************************************************************/
    private lateinit var currentOpMode: OpMode
    private val runtime = ElapsedTime()
    private val driveBase = MecanumDrive()
    private var currentGrabSystemState = GrabSystemState.STOWED
    private var timerActive = false
    private var timerExpiration = 0.0

    /***********************************************************************************************
     * HARDWARE MAPPINGS ***************************************************************************
     ***********************************************************************************************/
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

    private lateinit var blinkin: RevBlinkinLedDriver

    fun init(opMode: OpMode) {
        currentOpMode = opMode
        if (!(this::imu.isInitialized)) {
            // MAP HARDWARE
            imu             = currentOpMode.hardwareMap.get(IMU::class.java, "imu")

            rightFront      = currentOpMode.hardwareMap.get(DcMotor::class.java, "RightFront")
            leftFront       = currentOpMode.hardwareMap.get(DcMotor::class.java, "LeftFront")
            leftRear        = currentOpMode.hardwareMap.get(DcMotor::class.java, "LeftRear")
            rightRear       = currentOpMode.hardwareMap.get(DcMotor::class.java, "RightRear")

            armRotor        = currentOpMode.hardwareMap.get(DcMotor::class.java, "ArmRotor")
            armExtender     = currentOpMode.hardwareMap.get(DcMotor::class.java, "ArmExtender")

            compassServo    = currentOpMode.hardwareMap.get(CRServo::class.java, "CompassServo")
            compassEncoder  = currentOpMode.hardwareMap.get(DcMotor::class.java, "CompassEncoder")

            clawJoint       = currentOpMode.hardwareMap.get(Servo::class.java, "ClawJointServo")
            claw            = currentOpMode.hardwareMap.get(Servo::class.java, "ClawServo")
            pixelScoop      = currentOpMode.hardwareMap.get(Servo::class.java, "PixelScoopServo")
            airplaneLauncher = currentOpMode.hardwareMap.get(Servo::class.java, "AirplaneLauncherServo")

            extensionLimit  = currentOpMode.hardwareMap.get(TouchSensor::class.java, "ExtensionLimit")

            blinkin = currentOpMode.hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkin")

            // CONFIGURE MOTORS
            rightFront.direction  = DcMotorSimple.Direction.FORWARD
            leftFront.direction   = DcMotorSimple.Direction.FORWARD
            leftRear.direction    = DcMotorSimple.Direction.FORWARD
            rightRear.direction   = DcMotorSimple.Direction.FORWARD

            rightFront.zeroPowerBehavior    = DcMotor.ZeroPowerBehavior.BRAKE
            leftFront.zeroPowerBehavior     = DcMotor.ZeroPowerBehavior.BRAKE
            leftRear.zeroPowerBehavior      = DcMotor.ZeroPowerBehavior.BRAKE
            rightRear.zeroPowerBehavior     = DcMotor.ZeroPowerBehavior.BRAKE

            armRotor.direction    = DcMotorSimple.Direction.FORWARD
            armExtender.direction = DcMotorSimple.Direction.FORWARD

            compassEncoder.direction = DcMotorSimple.Direction.FORWARD

            // INITIALIZE IMU
            resetIMU()

            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK)
        }
        currentGrabSystemState = GrabSystemState.STOWED

        airplaneLauncher.position = LauncherPosition.LOADED.position
        pixelScoop.position = currentGrabSystemState.scoopTarget.position
        clawJoint.position = currentGrabSystemState.jointTarget.position
        claw.position = currentGrabSystemState.clawTarget.position
    }

    fun start() {
        resetYaw()
        setDefaultMotorModes()
        runtime.reset()
    }

    fun update() {
        if(timerActive && timerExpiration <= runtime.milliseconds()) {
            timerActive = false
            invokeCommand(ArmCommand.TIMEOUT)
        }

        if(extensionLimit.isPressed) {
            invokeCommand(ArmCommand.LIMIT_REACHED)
        }

        robotOrientation = imu.robotYawPitchRollAngles
        driveBase.yaw = robotOrientation.getYaw(AngleUnit.RADIANS)

        compass.target = -driveBase.totalYaw
        compass.update(compassEncoder.currentPosition)
        compassServo.power = compass.idealPower

        rightFront.power    = driveBase.powerFrontRight
        leftFront.power     = driveBase.powerFrontLeft
        leftRear.power      = driveBase.powerRearLeft
        rightRear.power     = driveBase.powerRearRight

        armRotor.targetPosition = currentGrabSystemState.rotorTarget.position
        armExtender.targetPosition = currentGrabSystemState.extenderTarget.position

        armRotor.power = ARM_ROTOR_SPEED
        armExtender.power = ARM_EXTENDER_SPEED

        pixelScoop.position = currentGrabSystemState.scoopTarget.position
        clawJoint.position = currentGrabSystemState.jointTarget.position
        claw.position = currentGrabSystemState.clawTarget.position

    }

    fun setDrivePower (speedRight: Double, speedForward: Double, speedTurn: Double) {
        driveBase.calculateDrivePower(speedRight, speedForward, speedTurn)
    }

    fun trimYaw (deltaYawTrim: Double) {
        driveBase.yawTrim += deltaYawTrim
    }

    fun launchPlaneIfEndgame () {
        if(runtime.seconds() > 60){
            airplaneLauncher.position = LauncherPosition.LAUNCHED.position
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE)
        }
    }

    fun resetYaw() {
        resetIMU()
        driveBase.resetYaw()
    }

    fun printToTelemetry() {
        currentOpMode.telemetry.addData("Status", "Run Time: $runtime")
        currentOpMode.telemetry.addData("Grabber State", currentGrabSystemState)

        currentOpMode.telemetry.addData("Motors", "")
        currentOpMode.telemetry.addData("", "(%.2f) (%.2f)", driveBase.powerFrontLeft, driveBase.powerFrontRight)
        currentOpMode.telemetry.addData("", "(%.2f) (%.2f)", driveBase.powerRearLeft, driveBase.powerRearRight)
    }

    fun resetIMU() {
        imu.initialize(IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)))

        imu.resetYaw()
    }

    fun stopAndResetAllEncoders() {
        rightFront.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftFront.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftRear.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightRear.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        armRotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armExtender.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        compassEncoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        compassServo.power = 0.0
    }

    fun setDefaultMotorModes() {
        rightFront.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftFront.mode = DcMotor.RunMode.RUN_USING_ENCODER
        leftRear.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightRear.mode = DcMotor.RunMode.RUN_USING_ENCODER

        armRotor.targetPosition = currentGrabSystemState.rotorTarget.position
        armExtender.targetPosition = currentGrabSystemState.extenderTarget.position
        armRotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        armExtender.mode = DcMotor.RunMode.RUN_TO_POSITION

        compassEncoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    /***********************************************************************************************
     * GRAB SYSTEM KINEMATIC STATE MACHINE *********************************************************
     ***********************************************************************************************/
    fun invokeCommand(command : ArmCommand) {
        val nextState : GrabSystemState? = commandMap.getValue(command)[currentGrabSystemState]

        if(nextState != null) {
            if(command == ArmCommand.LIMIT_REACHED) {
                armExtender.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                armExtender.mode = DcMotor.RunMode.RUN_USING_ENCODER
            }

            if(nextState == GrabSystemState.LIFTING || nextState == GrabSystemState.LOWERING) {
                timerActive = true
                timerExpiration = runtime.milliseconds() + 3500
            } else if(nextState == GrabSystemState.GRAB_READY) {
                timerActive = true
                timerExpiration = runtime.milliseconds() + 800
            } else if(nextState == GrabSystemState.GRABBING) {
                timerActive = true
                timerExpiration = runtime.milliseconds() + 1000
            }
        }

        currentGrabSystemState = nextState ?: currentGrabSystemState
    }

    enum class ArmCommand {
        STOW, SCOOP, GRAB, LIFT, DROP, PLACE, TIMEOUT, LIMIT_REACHED
    }
    enum class GrabSystemState (override val rotorTarget: RotorPosition,
                                override val extenderTarget: ExtenderPosition,
                                override val jointTarget: JointPosition,
                                override val scoopTarget: ScoopPosition,
                                override val clawTarget: ClawPosition) : GrabSystemPosition {
        STOWED (
            RotorPosition.STOWED,
            ExtenderPosition.MIN,
            JointPosition.FOLDED,
            ScoopPosition.UP,
            ClawPosition.CLOSED
        ) {
            override fun toString() = "STOWED"
          },
        LIFT_READY (
            RotorPosition.UP,
            ExtenderPosition.MAX,
            JointPosition.FOLDED,
            ScoopPosition.UP,
            ClawPosition.CLOSED
        ) {
            override fun toString() ="LIFT_READY"
        },
        LIFTING (
            RotorPosition.UP,
            ExtenderPosition.MIN,
            JointPosition.FOLDED,
            ScoopPosition.UP,
            ClawPosition.CLOSED
        ) {
            override fun toString() = "LIFTING"
        },
        LIFT_ACTIVE (
            RotorPosition.UP,
            ExtenderPosition.MIN,
            JointPosition.STRAIGHT,
            ScoopPosition.UP,
            ClawPosition.CLOSED
        ) {
            override fun toString() = "LIFT_ACTIVE"
        },
        LOWERING (
            RotorPosition.UP,
            ExtenderPosition.MAX,
            JointPosition.FOLDED,
            ScoopPosition.UP,
            ClawPosition.CLOSED
        ) {
            override fun toString() = "LOWERING"
        },
        SCOOP_READY (
            RotorPosition.FORWARD,
            ExtenderPosition.GRAB,
            JointPosition.FOLDED,
            ScoopPosition.DOWN,
            ClawPosition.OPEN
        ) {
            override fun toString() = "SCOOP_READY"
        },
        GRAB_READY (
            RotorPosition.GRAB,
            ExtenderPosition.GRAB,
            JointPosition.FOLDED,
            ScoopPosition.DOWN,
            ClawPosition.OPEN
        ) {
            override fun toString() = "GRAB_READY"
        },
        GRABBING (
            RotorPosition.GRAB,
            ExtenderPosition.GRAB,
            JointPosition.FOLDED,
            ScoopPosition.UP,
            ClawPosition.CLOSED
        ) {
            override fun toString() = "GRABBING"
        },
        GRABBED (
            RotorPosition.FORWARD,
            ExtenderPosition.GRAB,
            JointPosition.FOLDED,
            ScoopPosition.UP,
            ClawPosition.CLOSED
        ) {
            override fun toString() = "GRABBED"
        },
        BACKBOARD_READY (
            RotorPosition.BACKBOARD,
            ExtenderPosition.MAX,
            JointPosition.STRAIGHT,
            ScoopPosition.UP,
            ClawPosition.CLOSED
        ) {
            override fun toString() = "BACKBOARD_READY"
        },
        BACKBOARD_DROPPED (
            RotorPosition.BACKBOARD,
            ExtenderPosition.MAX,
            JointPosition.STRAIGHT,
            ScoopPosition.UP,
            ClawPosition.OPEN
        ) {
            override fun toString() = "BACKBOARD_DROPPED"
        };
    }

    private val commandMap = mapOf(
        ArmCommand.STOW to mapOf(
            GrabSystemState.BACKBOARD_READY to GrabSystemState.STOWED,
            GrabSystemState.BACKBOARD_DROPPED to GrabSystemState.STOWED,
            GrabSystemState.GRABBED to GrabSystemState.STOWED,
            GrabSystemState.SCOOP_READY to GrabSystemState.STOWED,
            GrabSystemState.LIFT_READY to GrabSystemState.STOWED
        ),
        ArmCommand.SCOOP to mapOf(
            GrabSystemState.STOWED to GrabSystemState.SCOOP_READY,
            GrabSystemState.GRABBED to GrabSystemState.SCOOP_READY,
            GrabSystemState.BACKBOARD_DROPPED to GrabSystemState.SCOOP_READY,
            GrabSystemState.LIFT_READY to GrabSystemState.SCOOP_READY
        ),
        ArmCommand.LIFT to mapOf(
            GrabSystemState.STOWED to GrabSystemState.LIFT_READY,
            GrabSystemState.GRABBED to GrabSystemState.LIFT_READY,
            GrabSystemState.SCOOP_READY to GrabSystemState.LIFT_READY,
            GrabSystemState.BACKBOARD_READY to GrabSystemState.LIFT_READY,
            GrabSystemState.BACKBOARD_DROPPED to GrabSystemState.LIFT_READY,
            GrabSystemState.LIFT_READY to GrabSystemState.LIFTING,
            GrabSystemState.LOWERING to GrabSystemState.LIFTING
        ),
        ArmCommand.DROP to mapOf(
            GrabSystemState.LIFTING to GrabSystemState.LOWERING,
            GrabSystemState.LIFT_ACTIVE to GrabSystemState.LOWERING
        ),
        ArmCommand.PLACE to mapOf(
            GrabSystemState.GRABBED to GrabSystemState.BACKBOARD_READY,
            GrabSystemState.SCOOP_READY to GrabSystemState.BACKBOARD_READY
        ),
        ArmCommand.GRAB to mapOf(
            GrabSystemState.SCOOP_READY to GrabSystemState.GRAB_READY,
            GrabSystemState.BACKBOARD_READY to GrabSystemState.BACKBOARD_DROPPED
        ),
        ArmCommand.TIMEOUT to mapOf(
            GrabSystemState.GRAB_READY to GrabSystemState.GRABBING,
            GrabSystemState.GRABBING to GrabSystemState.GRABBED,
            GrabSystemState.LOWERING to GrabSystemState.LIFT_READY,
            GrabSystemState.LIFTING to GrabSystemState.LIFT_ACTIVE
        ),
        ArmCommand.LIMIT_REACHED to mapOf(
            GrabSystemState.LIFTING to GrabSystemState.LIFT_ACTIVE
        )
    )

    fun SetTeamColor(isRed : Boolean) {
        if(isRed) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED)
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE)
        }
    }

    interface GrabSystemPosition {
        val rotorTarget: RotorPosition
        val extenderTarget: ExtenderPosition
        val jointTarget: JointPosition
        val scoopTarget: ScoopPosition
        val clawTarget: ClawPosition
    }

    interface DcMotorPosition {
        val position: Int
    }

    interface ServoPosition {
        val position: Double
    }
}