package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Left Stick - Normal tank scheme
// Right Stick Y - Lift up/down
// DPad - Going from side to side (and front too)
// Left Trigger - Grabber close
// Right Trigger - Grabber open

@TeleOp(name="2022-23 Manual", group="Exercises")
public class HumanMode extends LinearOpMode
{
    double leftStickX, leftStickY, rightStickX, rightStickY, flPower, frPower, blPower, brPower, liftPower;

    // Calculate the COUNTS_PER_INCH for your specific drive train
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
    static final double     SPEED                   = 1.5;
    // static final double     SPRINT_SPEED            = 3.0;
    static final double     LIFT_SPEED              = 1.5;
    static final double     GRABBER_SPEED           = 10.0;
    static final double     TIMEOUT                 = 2.0;

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

        double newFLPosition, newFRPosition, newBLPosition, newBRPosition, newLiftPosition;

        deltaTimer.reset();
        while (opModeIsActive())
        {
            deltaTime = deltaTimer.seconds();
            deltaTimer.reset();

            // Getting sticks values
            leftStickX  = gamepad1.left_stick_x;
            leftStickY  = -gamepad1.left_stick_y;
            rightStickX = -gamepad1.right_stick_x;
            rightStickY = -gamepad1.right_stick_y;

            telemetry.addData("Left Stick at",  leftStickX + ":" + leftStickY);
            telemetry.addData("Right Stick at", rightStickX + ":" + rightStickY);

            flPower = frPower = blPower = brPower = 0f;
            if (gamepad1.dpad_up) {
                flPower += 1f;
                frPower += 1f;
                blPower += 1f;
                brPower += 1f;
            }
            if (gamepad1.dpad_right) {
                flPower += 1f;
                frPower -= 1f;
                blPower -= 1f;
                brPower += 1f;
            }
            if (gamepad1.dpad_down) {
                flPower -= 1f;
                frPower -= 1f;
                blPower -= 1f;
                brPower -= 1f;
            }
            if (gamepad1.dpad_left) {
                flPower -= 1f;
                frPower += 1f;
                blPower += 1f;
                brPower -= 1f;
            }
            flPower += leftStickY + leftStickX;
            frPower += leftStickY - leftStickX;
            blPower += leftStickY + leftStickX;
            brPower += leftStickY - leftStickX;

            liftPower = -rightStickY;

            controller.addMove(new RobotController.Move(flPower, frPower, blPower, brPower, liftPower, gamepad1.left_trigger, 0));
            controller.flush();

            telemetry.update();

            idle();
        }
    }

    public boolean isActive() {
        return opModeIsActive();
    }
}