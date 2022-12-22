package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="C06 Drive Arcade .New", group="Exercises")
//@Disabled
public class CodeTestThing extends LinearOpMode
{
    // DigitalChannel armButton;
    DcMotor frontleftMotor, frontrightMotor, backleftMotor,backrightMotor, armMotor;
    float   backleftPower, backrightPower,frontleftPower, frontrightPower, armPower, xValue, yValue;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 288 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 98/25.4 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    ElapsedTime runtime = new ElapsedTime();

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {  
        frontleftMotor = hardwareMap.dcMotor.get("front_left_motor"); //phones 1
        frontrightMotor = hardwareMap.dcMotor.get("front_right_motor");
        backleftMotor = hardwareMap.dcMotor.get("back_left_motor");
        backrightMotor = hardwareMap.dcMotor.get("back_right_motor");
        armMotor = hardwareMap.dcMotor.get("arm_motor");

        // armButton = hardwareMap.digitalChannel.get("arm_button"); // find out how to initialize this

        frontleftMotor.setDirection(DcMotor.Direction.REVERSE);
        backleftMotor.setDirection(DcMotor.Direction.REVERSE);

        //initializing encoder for arm
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            //encoder movements for arm
            //reset arm
            if(gamepad1.a)
            {
                //reset arm to 0
                encoder_lift(.2, 0, 5);
            }
            else if(gamepad1.x)
            {
                //set arm to small pole height
                encoder_lift(.2, 14, 5);
            }
            else if(gamepad1.y)
            {
                //set arm to medium pole height
                encoder_lift(.2, 24, 5);
            }
            else if(gamepad1.b)
            {
                //set arm to tall pole height
                encoder_lift(.2, 34, 5);
            }

            //telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.addData("Currently at",  " at %7d :%7d",
                    frontleftMotor.getCurrentPosition(), frontrightMotor.getCurrentPosition());
            telemetry.addData("Currently at",  " at %7d :%7d",
                    backleftMotor.getCurrentPosition(), backrightMotor.getCurrentPosition());

            telemetry.update();

            idle();
        }
    }
    public void encoder_lift(double speed,
                             int inches,
                             double timeout)
    {
        double new_arm_pos;
        int current_pos = armMotor.getCurrentPosition();

        //makes sure op mode is A ok
        //#Lake Pride!
        if(opModeIsActive())
        {
            //position to move to
            armMotor.setTargetPosition(inches);

            //turns on RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //reset time
            runtime.reset();

            //Starts motion
            armMotor.setPower((int)(speed * COUNTS_PER_INCH));

            //Epic loop that continues while:
            // 1. Opmode is active
            // 2. Motors are running
            // 3. There is time left
            while(opModeIsActive() && (runtime.seconds() < timeout) && armMotor.isBusy())
            {
                //displays data to the driver
                telemetry.addData("Moving arm to",
                        " %7d",
                        inches);
                telemetry.addData("Current arm pos",
                        " %7d",
                        armMotor.getCurrentPosition());

                telemetry.update();
            }
//            if(armButton.getState() && inches == 0)
//            {
//                break;
//            }

            //Stops motion
            armMotor.setPower(0);

            //Turns off RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}