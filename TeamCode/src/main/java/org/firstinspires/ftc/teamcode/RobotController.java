package org.firstinspires.ftc.teamcode;

import androidx.annotation.FloatRange;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public class RobotController {
    public static class Move {
        public final double flPower, frPower, blPower, brPower;
        public final double liftPosition;
        @FloatRange(from=0.0f, to=1.0f)
        public final double grabberPosition;
        public final double waitFor;

        public Move(double flPower, double frPower, double blPower, double brPower, double liftPosition, double grabberPosition, double waitFor) {
            this.flPower = flPower;
            this.frPower = frPower;
            this.blPower = blPower;
            this.brPower = brPower;
            this.liftPosition = liftPosition;
            this.grabberPosition = grabberPosition;
            this.waitFor = waitFor;
        }
    }

    static final double     COUNTS_PER_MOTOR_REV    = 288;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 98/25.4;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     TIMEOUT                 = 5.0;

    double speed = 1.5;
    // double sprint_speed = 3.0;
    double lift_speed = 1.5;
    double grabber_speed = 10.0;

    final List<Move> moves = new ArrayList<>();

    final DistanceSensor LiftStopper;
    final Servo GrabberServo;
    final DcMotor FLMotor, FRMotor, BLMotor, BRMotor, LiftMotor;

    ElapsedTime runTimer = new ElapsedTime();

    final HumanMode main;

    public RobotController(HumanMode main) {
        this.main = main;

        LiftStopper     = main.hardwareMap.get(DistanceSensor.class, "lift_stopper");
        FLMotor         = main.hardwareMap.dcMotor.get("front_left_motor");
        FRMotor         = main.hardwareMap.dcMotor.get("front_right_motor");
        BLMotor         = main.hardwareMap.dcMotor.get("back_left_motor");
        BRMotor         = main.hardwareMap.dcMotor.get("back_right_motor");
        LiftMotor       = main.hardwareMap.dcMotor.get("lift_motor");
        GrabberServo    = main.hardwareMap.servo.get("grabber_servo");

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init() {
        this.setupArm();
    }

    public void addMove(Move move) {
        moves.add(move);
    }

    public void moveTo(@FloatRange(from=0, to=360) double direction, double amount) {
        double flPower = 0, frPower = 0, blPower = 0, brPower = 0;

        double x = Math.cos(direction), y = Math.sin(direction);
        flPower += (amount * y + amount * x);
        frPower += (amount * y - amount * x);
        blPower += (amount * y - amount * x);
        brPower += (amount * y + amount * x);

        this.moveFor(flPower, frPower, blPower, brPower);
    }

    public void moveFor(double flPower, double frPower, double blPower, double brPower) {
        addMove(new Move(
                flPower, frPower, blPower, brPower,
                LiftMotor.getCurrentPosition(),
                GrabberServo.getPosition(),
                0));
    }

    public void setLiftPosition(double position) {
        addMove(new Move(
                FLMotor.getCurrentPosition(),
                FRMotor.getCurrentPosition(),
                BLMotor.getCurrentPosition(),
                BRMotor.getCurrentPosition(),
                inchesToEncoderRots(position),
                GrabberServo.getPosition(),
                0));
    }

    public void setGrabberPosition(double position) {
        addMove(new Move(
                FLMotor.getCurrentPosition(),
                FRMotor.getCurrentPosition(),
                BLMotor.getCurrentPosition(),
                BRMotor.getCurrentPosition(),
                LiftMotor.getCurrentPosition(),
                position,
                0));
    }

    public void flush() {
        FLMotor.setPower(1.0);
        FRMotor.setPower(1.0);
        BLMotor.setPower(1.0);
        BRMotor.setPower(1.0);
        LiftMotor.setPower(1.0);
        for (Move move : moves) {
            FLMotor.setTargetPosition(FLMotor.getCurrentPosition() + inchesToEncoderRots(move.flPower));
            FRMotor.setTargetPosition(FRMotor.getCurrentPosition() + inchesToEncoderRots(move.frPower));
            BLMotor.setTargetPosition(BLMotor.getCurrentPosition() + inchesToEncoderRots(move.blPower));
            BRMotor.setTargetPosition(BRMotor.getCurrentPosition() + inchesToEncoderRots(move.brPower));
            LiftMotor.setTargetPosition(inchesToEncoderRots(move.liftPosition));

            GrabberServo.setPosition(move.grabberPosition);

            runTimer.reset();
            do {

            } while(main.isActive() &&
                    (runTimer.seconds() <= TIMEOUT) &&
                    (FLMotor.isBusy() || FRMotor.isBusy() || BLMotor.isBusy() || BLMotor.isBusy()) || LiftMotor.isBusy());

            runTimer.reset();
            do {

            } while (runTimer.seconds() <= move.waitFor);
        }
        moves.clear();
    }

    public void setSpeed(double speed) { this.speed = speed; }
    public void setLiftSpeed(double speed) { this.lift_speed = speed; }
    public void setGrabberSpeed(double speed) { this.grabber_speed = speed; }

    public double getSpeed() { return this.speed; }
    public double setLiftSpeed() { return this.lift_speed; }
    public double setGrabberSpeed() { return this.grabber_speed; }

    void setupArm() {
        boolean setting_up = LiftStopper.getDistance(DistanceUnit.INCH) < 1;
        LiftMotor.setPower(lift_speed);

        while (setting_up) {
            if (LiftStopper.getDistance(DistanceUnit.INCH) < 1) setting_up = false;
            LiftMotor.setTargetPosition(LiftMotor.getTargetPosition() - 1);
        }

        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    int inchesToEncoderRots(double inches) { return (int)(inches * COUNTS_PER_INCH); }
    double encoderRotsToInches(int rots) { return (double)(rots / COUNTS_PER_INCH); }
}
