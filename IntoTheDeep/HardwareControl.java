package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareControl {

    public final double MAX_VELOCITY = 2250;
    public final double MAX_ACCEL = 11250;

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx hang, liftShort, liftLong, ptoMotor;

    public Servo intakeArm, intakeWrist, intakeRotate, intakeClaw;
    public Servo outtakeArm, outtakeWrist, outtakeRotate, outtakeClaw;
    public Servo linkageLeft, linkageRight;
    public Servo ptoServo;

    public DigitalChannel intakeSwitch;

    public HardwareControl(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        hang = hardwareMap.get(DcMotorEx.class, "hang");
        liftShort = hardwareMap.get(DcMotorEx.class, "shortChain");
        liftLong = hardwareMap.get(DcMotorEx.class, "longChain");
        ptoMotor = hardwareMap.get(DcMotorEx.class, "ptoMotor");

        liftLong.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLong.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        linkageLeft = hardwareMap.get(Servo.class, "linkageLeft");
        linkageRight = hardwareMap.get(Servo.class, "linkageRight");

        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        outtakeArm = hardwareMap.get(Servo.class, "outtakeArm");
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeRotate = hardwareMap.get(Servo.class, "outtakeRotate");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

        ptoServo = hardwareMap.get(Servo.class, "ptoServo");

        intakeSwitch = hardwareMap.get(DigitalChannel.class, "magnetSwitch");

    }

    public boolean isIntakeIn() {
        return !intakeSwitch.getState();
    }

    public void drive(double x, double y, double rx) {
        rx *= -1;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void intakeBack() {
        rotateIntake(0);
        intakeArm.setPosition(0.8588);
        intakeWrist.setPosition(0);
    }

    public void clampOuttake() {
        outtakeClaw.setPosition(0.55);
    }

    public void openIntakeClaw() {
        intakeClaw.setPosition(0.14);
    }

    public void closeIntakeClaw() {
        intakeClaw.setPosition(0.31);
    }

    public void intakeHover() {
        intakeArm.setPosition(0.48);
        intakeWrist.setPosition(0.86);
    }

    public void intakeDown() {
        intakeArm.setPosition(0.388);
    }

    public void sampleGrabbedPosition() {
        intakeArm.setPosition(0.4284);
        intakeWrist.setPosition(0.58);
    }

    public void intakeTransfer() {
        intakeArm.setPosition(0.8588);
        intakeWrist.setPosition(0.3);
        intakeRotate.setPosition(0.68);
    }

    public void rotateIntake(double angle) {
        intakeRotate.setPosition(0.33 - 0.66 / 180.0 * (angle - 90)); // move 0 position closer to transfer
    }

    public void intakeLinkageOut() {
        linkageLeft.setPosition(0.92);
        linkageRight.setPosition(0.058);
    }

    public void intakeLinkageIn() {
        linkageLeft.setPosition(0.58);
        linkageRight.setPosition(0.388);
    }

    public void openOuttakeClaw() {
        outtakeClaw.setPosition(0.302);
    }

    public void closeOuttakeClaw() {
        outtakeClaw.setPosition(0.5050);
    }

    public void outtakeTransfer() {
        outtakeArm.setPosition(0.73);
        outtakeWrist.setPosition(0.87);
        rotateOuttakeOne();
    }

    public void outtakeVertical() {
        outtakeArm.setPosition(0.48);
        outtakeWrist.setPosition(0.32);
        rotateHalfway();
    }

    public void outtakeScoreBucket() {
        outtakeArm.setPosition(0.48);
        outtakeWrist.setPosition(0.1);
        rotateHalfway();
    }

    public void rotateOuttakeOne() {
        outtakeRotate.setPosition(0.996);
    }

    public void rotateHalfway() {
        outtakeRotate.setPosition(0.933);
    }

    public int getLiftPosition() {
        return -liftLong.getCurrentPosition();
    }
    public double getLiftVelocity() { return -liftLong.getVelocity(); }

    public void specimenGrab() {
        outtakeArm.setPosition(0.96);
        outtakeWrist.setPosition(0.6);
        rotateOuttakeOne();
    }

    public void specimenOut() {
        outtakeArm.setPosition(0.09);
        outtakeWrist.setPosition(0.36);
    }

    public void specimenScore() {
        outtakeArm.setPosition(0.16);
        outtakeWrist.setPosition(0.36);
    }

    public void setLiftPower(double power) {
        liftLong.setPower(-power);
        liftShort.setPower(power);
    }

    public void outtakeHang() {
        outtakeArm.setPosition(0.35);
        outtakeWrist.setPosition(0.2);
    }

    public void resetLiftEncoder() {
        liftLong.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLong.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHangPower(double power) {
        hang.setPower(power);
    }

    public int getHangPosition() {
        return hang.getCurrentPosition();
    }

    public void ptoRelease() {
        ptoServo.setPosition(1);
    }

    public void ptoLock() {
        ptoServo.setPosition(0.54);
    }

    public int getLiftPosTarget(double seconds) {
        if (seconds < 0.2) {
            return 5625 * Math.pow(seconds, 2);
        } else if (seconds < 0.5) {
            return 225 + 3375 * (seconds - 0.2);
        } else {
            return 900 + 3375 * (seconds - 0.2) - 5625 * Math.pow((seconds - 0.5), 2);
        }
    }

    public int getLiftVelTarget(double seconds) {
        if (seconds < 0.2) {
            return 1125 * seconds;
        } else if (seconds < 0.5) {
            return 2250;
        } else {
            return 7875 - 11250 * seconds;
        }
    }

}