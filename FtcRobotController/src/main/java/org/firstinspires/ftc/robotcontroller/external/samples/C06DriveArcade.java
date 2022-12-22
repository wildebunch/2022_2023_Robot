//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="C06 Drive Arcade .New", group="Exercises")
//@Disabled
public class C06DriveArcade extends LinearOpMode
{
    DcMotor frontleftMotor, frontrightMotor, backleftMotor,backrightMotor, armMotor;
    float   backleftPower, backrightPower,frontleftPower, frontrightPower, armPower, xValue, yValue;


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        frontleftMotor = hardwareMap.dcMotor.get("FL");
        frontrightMotor = hardwareMap.dcMotor.get("FR");
        backleftMotor = hardwareMap.dcMotor.get("BL");
        backrightMotor = hardwareMap.dcMotor.get("BR");
        armMotor = hardwareMap.dcMotor.get("grabber");


        backleftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        while (opModeIsActive())
        {
            yValue = gamepad1.right_stick_y * -1;
            xValue = gamepad1.right_stick_x * -1;

            backleftPower =  yValue - xValue;
            backrightPower = yValue + xValue;
            frontrightPower = yValue + xValue;
            frontleftPower = yValue - xValue;

            backleftMotor.setPower(Range.clip(backleftPower, -1.0, 1.0));
            backrightMotor.setPower(Range.clip(backrightPower, -1.0, 1.0));
            frontleftMotor.setPower(Range.clip(frontleftPower, -1.0, 1.0));
            frontrightMotor.setPower(Range.clip(frontrightPower, -1.0, 1.0));


            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + frontleftPower + "  right=" + frontrightPower);
            telemetry.update();

            idle();
        }
    }
}