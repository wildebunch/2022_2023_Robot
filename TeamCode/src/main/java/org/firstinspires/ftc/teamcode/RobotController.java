package org.firstinspires.ftc.teamcode;

import android.app.ActivityManager;

import androidx.annotation.FloatRange;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.checkerframework.checker.regex.qual.Regex;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.math.MathContext;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.regex.Pattern;

public class RobotController {
    private static final double     COUNTS_PER_MOTOR_REV    = 288;
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    private static final double     WHEEL_DIAMETER_INCHES   = 4;
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    private static final double     TIMEOUT                 = 2.0;
    private static final double     SETUP_TIMEOUT           = 10.0;

    private static final double     GRABBER_CLOSED          = 0;
    private static final double     GRABBER_OPEN          = 0.5;

    private double speed = 1.5;
    // double sprint_speed = 3.0;
    private double lift_speed = 1.5;
    private double grabber_speed = 10.0;
    private double rotation_speed = 0.5;

    private final RevTouchSensor LiftStopper;
    private final Servo GrabberServo;
    private boolean _grabberState = false;
    private final DcMotor FLMotor, FRMotor, BLMotor, BRMotor, LiftMotor;
    private BNO055IMU imu;

    private ElapsedTime runTimer = new ElapsedTime();

    private final RobotOpMode main;

    private boolean isInit = false;

    private boolean _maintainAngle = false;
    private double angleToMaintain;

    public RobotController(RobotOpMode main) {
        this.main = main;

        LiftStopper     = main.hardwareMap.get(RevTouchSensor.class, "lift_stopper");
        FLMotor         = main.hardwareMap.dcMotor.get("front_left_motor");
        FRMotor         = main.hardwareMap.dcMotor.get("front_right_motor");
        BLMotor         = main.hardwareMap.dcMotor.get("back_left_motor");
        BRMotor         = main.hardwareMap.dcMotor.get("back_right_motor");
        LiftMotor       = main.hardwareMap.dcMotor.get("lift_motor");
        GrabberServo    = main.hardwareMap.servo.get("grabber_servo");
        imu             = main.hardwareMap.get(BNO055IMU.class, "imu");

        // IMU
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.mode = BNO055IMU.SensorMode.IMU;
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.loggingEnabled = false;
        imu.initialize(param);
        while (!main.isStopRequested() && !imu.isGyroCalibrated() && main.isActive())
        {
            main.sleep(50);
        }
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        main.sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        main.sleep(100); //Changing modes again requires a delay

        angleToMaintain = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).thirdAngle;

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void init() {
        this.setupArm();

        FLMotor.setPower(1.0);
        FRMotor.setPower(1.0);
        BLMotor.setPower(1.0);
        BRMotor.setPower(1.0);
        LiftMotor.setPower(1.0);

        FLMotor.setTargetPosition(0);
        FRMotor.setTargetPosition(0);
        BLMotor.setTargetPosition(0);
        BRMotor.setTargetPosition(0);
        LiftMotor.setTargetPosition(0);

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.isInit = true;
    }

    public void moveTo(int flPosition, int frPosition, int blPosition, int brPosition, int liftPosition, boolean grabberState, double waitFor) {
        if (!isInit)
            throw new RuntimeException("Controller should be init first.");

        FLMotor.setTargetPosition(FLMotor.getCurrentPosition() + (int)(flPosition * this.speed));
        FRMotor.setTargetPosition(FRMotor.getCurrentPosition() + (int)(frPosition * this.speed));
        BLMotor.setTargetPosition(BLMotor.getCurrentPosition() + (int)(blPosition * this.speed));
        BRMotor.setTargetPosition(BRMotor.getCurrentPosition() + (int)(brPosition * this.speed));
        LiftMotor.setTargetPosition(Math.min(Math.max(liftPosition, 0), 1599));

        GrabberServo.setPosition((grabberState ? GRABBER_OPEN : GRABBER_CLOSED));

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runTimer.reset();
        while (main.isActive() && runTimer.seconds() <= waitFor) {}
    }

    public void moveFor(double flPower, double frPower, double blPower, double brPower, double liftPower, boolean grabberState, double waitFor) {
        _grabberState = grabberState;
        int newListPosition = LiftMotor.getCurrentPosition() + inchesToEncoderRots(liftPower * this.lift_speed);
        if (liftPower == 0 && _setLiftPosition) {
            newListPosition = LiftMotor.getTargetPosition();
        } else _setLiftPosition = false;

        moveTo(
                inchesToEncoderRots(flPower), inchesToEncoderRots(frPower), inchesToEncoderRots(blPower), inchesToEncoderRots(brPower),
                newListPosition,
                grabberState,
                waitFor
        );
    }

    private boolean _setLiftPosition = false;
    public void setLiftPosition(double height) {
        _setLiftPosition = true;
        LiftMotor.setTargetPosition(inchesToEncoderRots(height));
    }

    // Automation
    public void grabber(boolean state) {
        GrabberServo.setPosition((state? GRABBER_OPEN : GRABBER_CLOSED));
    }

    public void lift(double height) {
        setLiftPosition(height);
        do { } while (main.isActive() && LiftMotor.isBusy());
    }

    public void forward(double inches) {
        int steps = inchesToEncoderRots(inches);

        FLMotor.setPower(this.speed);
        FRMotor.setPower(this.speed);
        BLMotor.setPower(this.speed);
        BRMotor.setPower(this.speed);

        FLMotor.setTargetPosition(FLMotor.getCurrentPosition() - steps);
        FRMotor.setTargetPosition(FRMotor.getCurrentPosition() + steps);
        BLMotor.setTargetPosition(BLMotor.getCurrentPosition() - steps);
        BRMotor.setTargetPosition(BRMotor.getCurrentPosition() + steps);

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        do { } while(main.isActive() && (FLMotor.isBusy() || FRMotor.isBusy() || BLMotor.isBusy() || BRMotor.isBusy()));
    }

    public void rotate(double angle) {
        angleToMaintain = getAngle(angleToMaintain + angle);
        align();
    }

    public void backward(double amount) { forward(-amount); }

    public void maintainAngle(boolean state) {
        this._maintainAngle = state;
        if (this._maintainAngle) {
            angleToMaintain = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).thirdAngle;
        }
    }

    public void align() {
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double power, deltaAngle;
        Orientation o;

        do {
            o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
            main.telemetry.addData("M", angleToMaintain + "");
            main.telemetry.addData("A", o.thirdAngle);
            deltaAngle = getAngle(angleToMaintain - o.thirdAngle);

            power = Math.max(Math.min(deltaAngle, 1), -1) * this.rotation_speed;

            FLMotor.setPower(power);
            FRMotor.setPower(power);
            BLMotor.setPower(power);
            BRMotor.setPower(power);

            main.telemetry.addData("Delta", "" + deltaAngle);
            main.telemetry.update();
        } while (main.isActive() && Math.abs(deltaAngle) >= 0.5);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
    }

    public void setSpeed(double speed) { this.speed = speed; }
    public void setLiftSpeed(double speed) { this.lift_speed = speed; }
    public void setGrabberSpeed(double speed) { this.grabber_speed = speed; }
    public void setRotationSpeed(double speed) { this.rotation_speed = speed; }

    public double getSpeed() { return this.speed; }
    public double getLiftSpeed() { return this.lift_speed; }
    public double getGrabberSpeed() { return this.grabber_speed; }
    public double getRotationSpeed() { return this.rotation_speed; }

    public void setupArm() {
        LiftMotor.setPower(1.0);
        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (!LiftStopper.isPressed() && main.isActive()) {
            LiftMotor.setTargetPosition(LiftMotor.getTargetPosition() - inchesToEncoderRots(1));
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runTimer.reset();
            do { } while(!LiftStopper.isPressed() && main.isActive() && LiftMotor.isBusy() && runTimer.seconds() < TIMEOUT);
        }

        LiftMotor.setTargetPosition(LiftMotor.getTargetPosition() + 10);
        do { } while(main.isActive() && LiftMotor.isBusy() && runTimer.seconds() < TIMEOUT);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isBusy() {
        return FLMotor.isBusy() || FRMotor.isBusy() || BLMotor.isBusy() || BRMotor.isBusy() || LiftMotor.isBusy();
    }

    public void waitWhileBusy() {
        while (main.isActive() && isBusy());
    }

    int inchesToEncoderRots(double inches) { return (int)(inches * COUNTS_PER_INCH); }
    double encoderRotsToInches(int rots) { return (double)(rots / COUNTS_PER_INCH); }

    boolean motorIsBusy(DcMotor motor) {
        final int gap = 10;
        // return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < gap;
        return motor.isBusy();
    }

    private double getAngle(double angle) {
        if (angle > 180)
            return angle - 360;
        else if (angle < -180)
            return angle + 360;
        return angle;
    }
}
