package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// Left Stick - Normal tank scheme
// Right Stick Y - Lift up/down
// DPad - Going from side to side (and front too)
// Left Trigger - Grabber close
// Right Trigger - Grabber open

@TeleOp(name="2022-23 Debug", group="")
public class DebugMode extends RobotOpMode
{
    private static final double     COUNTS_PER_MOTOR_REV    = 288;
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    private static final double     WHEEL_DIAMETER_INCHES   = 4;
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    double leftStickX, leftStickY, rightStickX, rightStickY, flPower, frPower, blPower, brPower, liftPower;
    static final double     SPEED                   = 1.5;
    static final double     DPAD_SPEED              = 3.0;
    static final double     LIFT_SPEED              = 1.5;
    static final double     GRABBER_SPEED           = 10.0;

    ElapsedTime runTimer = new ElapsedTime();
    ElapsedTime deltaTimer = new ElapsedTime();
    double deltaTime = 0;

    double FLOffset, FROffset, BLOffset, BROffset;

    @Override
    public void runOpMode()
    {
        RobotController controller = new RobotController(this);

        final RevTouchSensor LiftStopper;
        final Servo GrabberServo;
        final DcMotor FLMotor, FRMotor, BLMotor, BRMotor, LiftMotor;

        LiftStopper     = hardwareMap.get(RevTouchSensor.class, "lift_stopper");
        FLMotor         = hardwareMap.dcMotor.get("front_left_motor");
        FRMotor         = hardwareMap.dcMotor.get("front_right_motor");
        BLMotor         = hardwareMap.dcMotor.get("back_left_motor");
        BRMotor         = hardwareMap.dcMotor.get("back_right_motor");
        LiftMotor       = hardwareMap.dcMotor.get("lift_motor");
        GrabberServo    = hardwareMap.servo.get("grabber_servo");

        // wait for start button.
        telemetry.addData("Mode", "waiting for start...");
        telemetry.update();
        waitForStart();

        controller.init();

        deltaTimer.reset();
        while (isActive())
        {
            deltaTime = deltaTimer.seconds();
            deltaTimer.reset();

            // Getting sticks values
            leftStickX  = -gamepad1.left_stick_x;
            leftStickY  = -gamepad1.left_stick_y;
            rightStickX = -gamepad1.right_stick_x;
            rightStickY = -gamepad1.right_stick_y;

            telemetry.addData("Left Stick at",  leftStickX + ":" + leftStickY);
            telemetry.addData("Right Stick at", rightStickX + ":" + rightStickY);

            flPower = frPower = blPower = brPower = 0f;
            controller.setSpeed(1);
            flPower = -leftStickY;
            frPower = leftStickY;
            blPower = -leftStickY;
            brPower = leftStickY;

            liftPower = rightStickY;
            controller.setLiftSpeed(1);
            if (liftPower < 0) {
                controller.setLiftSpeed(0.75);
            }

            controller.moveFor(flPower, frPower, blPower, brPower, liftPower, gamepad1.right_bumper, 0);

            telemetry.addData("FL", (encoderRotsToInches(FLMotor.getCurrentPosition()) - FLOffset) + "");
            telemetry.addData("FR", (encoderRotsToInches(FRMotor.getCurrentPosition()) - FROffset) + "");
            telemetry.addData("BL", (encoderRotsToInches(BLMotor.getCurrentPosition()) - BLOffset) + "");
            telemetry.addData("BR", (encoderRotsToInches(BRMotor.getCurrentPosition()) - BROffset) + "");
            telemetry.addData("LIFT", encoderRotsToInches(LiftMotor.getCurrentPosition()));
            telemetry.addData("GRABBER", (gamepad1.right_bumper ? "PRESSED" : "UNPRESSED"));
            telemetry.update();

            if (gamepad1.guide) {
                FLOffset = encoderRotsToInches(FLMotor.getCurrentPosition());
                FROffset = encoderRotsToInches(FRMotor.getCurrentPosition());
                BLOffset = encoderRotsToInches(BLMotor.getCurrentPosition());
                BROffset = encoderRotsToInches(BRMotor.getCurrentPosition());
            }
            if (gamepad1.right_trigger > 0.5) {
                controller.rotate(45);
            }
            else if (gamepad1.left_trigger > 0.5) {
                controller.rotate(-45);
            }

            idle();
        }
    }

    int inchesToEncoderRots(double inches) { return (int)(inches * COUNTS_PER_INCH); }
    double encoderRotsToInches(int rots) { return (double)(rots / COUNTS_PER_INCH); }
}