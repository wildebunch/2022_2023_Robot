package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Left Stick - Normal tank scheme
// Right Stick Y - Lift up/down
// DPad - Going from side to side (and front too)
// Left Trigger - Grabber close
// Right Trigger - Grabber open

@TeleOp(name="2022-23 Manual", group="")
public class HumanMode extends RobotOpMode
{
    double leftStickX, leftStickY, rightStickX, rightStickY, flPower, frPower, blPower, brPower, liftPower;
    static final double     SPEED                   = 1.5;
    static final double     DPAD_SPEED              = 3.0;
    static final double     LIFT_SPEED              = 1.5;
    static final double     GRABBER_SPEED           = 10.0;

    ElapsedTime runTimer = new ElapsedTime();
    ElapsedTime deltaTimer = new ElapsedTime();
    double deltaTime = 0;

    @Override
    public void runOpMode()
    {
        RobotController controller = new RobotController(this);

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
            if (gamepad1.dpad_up) {
                flPower -= 1f;
                frPower += 1f;
                blPower -= 1f;
                brPower += 1f;
                controller.setSpeed(0.5);
            }
            if (gamepad1.dpad_right) {
                flPower -= 1f;
                frPower -= 1f;
                blPower += 1f;
                brPower += 1f;
                controller.setSpeed(0.5);
            }
            if (gamepad1.dpad_down) {
                flPower += 1f;
                frPower -= 1f;
                blPower += 1f;
                brPower -= 1f;
                controller.setSpeed(0.5);
            }
            if (gamepad1.dpad_left) {
                flPower += 1f;
                frPower += 1f;
                blPower -= 1f;
                brPower -= 1f;
                controller.setSpeed(0.5);
            }
            flPower += leftStickX - leftStickY;
            frPower += leftStickX + leftStickY;
            blPower += leftStickX - leftStickY;
            brPower += leftStickX + leftStickY;

            liftPower = rightStickY;
            controller.setLiftSpeed(1);
            if (liftPower < 0) {
                controller.setLiftSpeed(0.75);
            }

            if (gamepad1.a) {
                controller.setLiftPosition(2.78);
            }
            else if (gamepad1.b) {
                controller.setLiftPosition(31.55);
            }
            else if (gamepad1.x) {
                controller.setLiftPosition(54.24);
            }
            else if (gamepad1.y) {
                controller.setLiftPosition(69.77);
            }

            controller.moveFor(flPower, frPower, blPower, brPower, liftPower, gamepad1.right_bumper, 0);

            telemetry.update();

            idle();
        }
    }
}