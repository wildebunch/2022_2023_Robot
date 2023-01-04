package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="2022-2023 AutoMode", group="Exercises")
//@Disabled
public class AutomatedMode extends LinearOpMode
{
    DigitalChannel LiftButton;
    Servo GrabberServo;
    DcMotor FLMotor, FRMotor, BLMotor, BRMotor, LiftMotor;
    float leftStickX, leftStickY, rightStickX, rightStickY, flPower, frPower, blPower, brPower, liftPower;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 288;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 98/25.4;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     SPEED                   = 1.2;
    static final double     LIFT_SPEED              = 0.6;
    static final double     GRABBER_SPEED           = 10.0;
    static final double     TIMEOUT                 = 2.0;

    ElapsedTime runtime = new ElapsedTime();

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get hardware from config
        LiftButton      = hardwareMap.digitalChannel.get("lift_button");
        FLMotor         = hardwareMap.dcMotor.get("front_left_motor");
        FRMotor         = hardwareMap.dcMotor.get("front_right_motor");
        BLMotor         = hardwareMap.dcMotor.get("back_left_motor");
        BRMotor         = hardwareMap.dcMotor.get("back_right_motor");
        LiftMotor       = hardwareMap.dcMotor.get("lift_motor");
        GrabberServo    = hardwareMap.servo.get("grabber_servo");

        // Set motors modes for encoders
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // wait for start button.
        telemetry.addData("Mode", "waiting for start...");
        telemetry.update();
        waitForStart();

        int newFLPosition, newFRPosition, newBLPosition, newBRPosition;

        while (opModeIsActive())
        {
            // Getting sticks values
            leftStickX  = -gamepad1.left_stick_x;
            leftStickY  = gamepad1.left_stick_y;
            rightStickX = -gamepad1.right_stick_x;
            rightStickY = -gamepad1.right_stick_y;

            telemetry.addData("Left Stick at", "%7d:%7d", leftStickX, leftStickY);
            telemetry.addData("Right Stick at", "%7d:%7d", rightStickX, rightStickY);

            GrabberServo.setPosition(GrabberServo.getPosition() + (gamepad1.left_trigger - gamepad1.right_trigger) * GRABBER_SPEED);

            flPower = frPower = blPower = brPower = 0f;
            if (gamepad1.dpad_up) {
                flPower += 1f;
                frPower += 1f;
                blPower += 1f;
                brPower += 1f;
            }
            if (gamepad1.dpad_right) {
                flPower += 1f;
                frPower += -1f;
                blPower += 1f;
                brPower += -1f;
            }
            if (gamepad1.dpad_down) {
                flPower += -1f;
                frPower += -1f;
                blPower += -1f;
                brPower += -1f;
            }
            if (gamepad1.dpad_left) {
                flPower += -1f;
                frPower += 1f;
                blPower += -1f;
                brPower += 1f;
            }
            flPower += leftStickX + leftStickY;
            frPower += leftStickX - leftStickY;
            blPower += leftStickX + leftStickY;
            brPower += leftStickX - leftStickY;

            // liftPower = Range.clip(rightStickX, -1f, 1f);

            newFLPosition = FLMotor.getCurrentPosition() + inchesToEncoderRots(flPower);
            newFRPosition = FRMotor.getCurrentPosition() + inchesToEncoderRots(frPower);
            newBLPosition = BLMotor.getCurrentPosition() + inchesToEncoderRots(-blPower);
            newBRPosition = BRMotor.getCurrentPosition() + inchesToEncoderRots(-brPower);
            FLMotor.setTargetPosition(newFLPosition);
            FRMotor.setTargetPosition(newFRPosition);
            BLMotor.setTargetPosition(newBLPosition);
            BRMotor.setTargetPosition(newBRPosition);

            FLMotor.setPower(SPEED);
            FRMotor.setPower(SPEED);
            BLMotor.setPower(SPEED);
            BRMotor.setPower(SPEED);
            // LiftMotor.setPower(LIFT_SPEED);

            FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            while (opModeIsActive() &&
                    (runtime.seconds() <= TIMEOUT) &&
                    (FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BLMotor.isBusy())) {
                telemetry.addData("Currently at",
                        "FL: " + FLMotor.getCurrentPosition() +
                                "FR: " + FRMotor.getCurrentPosition() +
                                "BL: " + BLMotor.getCurrentPosition() +
                                "BR: " + BRMotor.getCurrentPosition()
                );
                telemetry.addData("Going to",
                        "FL: " + newFLPosition +
                                "FR: " + newFRPosition +
                                "BL: " + newBLPosition +
                                "BR: " + newBRPosition);
                telemetry.update();
            }

            telemetry.addData("Lift Button State: ", (LiftButton.getState() ? "on" : "off"));
            telemetry.update();

            idle();
        }
    }

    int inchesToEncoderRots(float inches) { return (int)(inches * COUNTS_PER_INCH); }
    float encoderRotsToInches(int rots) { return (float)(rots / COUNTS_PER_INCH); }
}