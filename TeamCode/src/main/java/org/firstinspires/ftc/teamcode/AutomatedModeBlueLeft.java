package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="2022-2023 Automated Blue Left", group="")
//@Disabled
public class AutomatedModeBlueLeft extends RobotOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotController controller = new RobotController(this);

        // wait for start button.
        telemetry.addData("Mode", "waiting for start...");
        telemetry.update();
        waitForStart();

        controller.init();

        controller.setSpeed(0.5);
        controller.setRotationSpeed(0.3);
        controller.setLiftSpeed(1);

        controller.grabber(true);
        controller.lift(32);
        controller.rotate(-40);
        controller.forward(9.0);
        controller.grabber(false);

        sleep(5000);

        idle();
        stop();
    }
}