package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="2022-2023 Automated Blue Right", group="")
//@Disabled
public class AutomatedModeBlueRight extends RobotOpMode
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
        controller.rotate(40);
        controller.forward(9.0);
        controller.grabber(false);
        controller.backward(9.0);
        controller.lift(0);
        controller.rotate(-135);
        controller.forward(27.3);

        sleep(5000);

        idle();
        stop();
    }
}