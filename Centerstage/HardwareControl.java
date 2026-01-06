package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class HardwareControl {

    DcMotor frontleft, frontright, backleft, backright, extension, lift, hook;
    DcMotorEx intake;
    Servo basketPivot, basketRotate, pixelA, pixelB, intakeLift, intakeGate, drone, hookLeft, hookRight, purpleServo;
    DigitalChannel intakeLiftSwitch, liftSwitch, extensionSwitch, beamA, beamB;
    BNO055IMU imu;

    double rotateTarget;

    public void initTele(HardwareMap hardwareMap) {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        extension = hardwareMap.get(DcMotor.class, "extension");
        lift = hardwareMap.get(DcMotor.class, "lift");
        hook = hardwareMap.get(DcMotor.class, "hook");
        hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        basketPivot = hardwareMap.get(Servo.class, "basketPivot");
        basketRotate = hardwareMap.get(Servo.class, "basketRotate");
        pixelA = hardwareMap.get(Servo.class, "topPixel");
        pixelB = hardwareMap.get(Servo.class, "bottomPixel");
        drone = hardwareMap.get(Servo.class, "drone");
        intakeLift = hardwareMap.get(Servo.class, "intakeLift");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");
        hookLeft = hardwareMap.get(Servo.class, "hookLeft");
        hookRight = hardwareMap.get(Servo.class, "hookRight");
        purpleServo = hardwareMap.get(Servo.class, "purpleServo");

        liftSwitch = hardwareMap.get(DigitalChannel.class, "liftSwitch");
        extensionSwitch = hardwareMap.get(DigitalChannel.class, "extensionSwitch");
        intakeLiftSwitch = hardwareMap.get(DigitalChannel.class, "intakeLiftSwitch");
        beamA = hardwareMap.get(DigitalChannel.class, "beamB");
        beamB = hardwareMap.get(DigitalChannel.class, "beamA");

        closeIntakeGate();
        basketIn();
        intakeLiftUp();
        basketRotate.setPosition(0.69);
        rotateTarget = 0.69;
        clampDrone();
//        hooksDown();
        releaseA();
        releaseB();
        releasePurple();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    // TODO: ENSURE THE INIT AUTO FUNCTION WORKS
    public void initAuto(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        extension = hardwareMap.get(DcMotor.class, "extension");
        lift = hardwareMap.get(DcMotor.class, "lift");
//        hook = hardwareMap.get(DcMotor.class, "hook");

        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        basketPivot = hardwareMap.get(Servo.class, "basketPivot");
        basketRotate = hardwareMap.get(Servo.class, "basketRotate");
        pixelA = hardwareMap.get(Servo.class, "topPixel");
        pixelB = hardwareMap.get(Servo.class, "bottomPixel");
        drone = hardwareMap.get(Servo.class, "drone");
        intakeLift = hardwareMap.get(Servo.class, "intakeLift");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");
        hookLeft = hardwareMap.get(Servo.class, "hookLeft");
        hookRight = hardwareMap.get(Servo.class, "hookRight");

        liftSwitch = hardwareMap.get(DigitalChannel.class, "liftSwitch");
        extensionSwitch = hardwareMap.get(DigitalChannel.class, "extensionSwitch");
        intakeLiftSwitch = hardwareMap.get(DigitalChannel.class, "intakeLiftSwitch");
        beamA = hardwareMap.get(DigitalChannel.class, "beamB");
        beamB = hardwareMap.get(DigitalChannel.class, "beamA");
        purpleServo = hardwareMap.get(Servo.class, "purpleServo");

        grabPurple();
        closeIntakeGate();
        basketIn();
        intakeLiftUp();
        basketRotate.setPosition(0.69);
        rotateTarget = 0.69;
//        hooksDown();
        releaseA();
    }

    public void initAutoReversedBasket(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        extension = hardwareMap.get(DcMotor.class, "extension");
        lift = hardwareMap.get(DcMotor.class, "lift");
//        hook = hardwareMap.get(DcMotor.class, "hook");

        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        basketPivot = hardwareMap.get(Servo.class, "basketPivot");
        basketRotate = hardwareMap.get(Servo.class, "basketRotate");
        pixelA = hardwareMap.get(Servo.class, "topPixel");
        pixelB = hardwareMap.get(Servo.class, "bottomPixel");
        drone = hardwareMap.get(Servo.class, "drone");
        intakeLift = hardwareMap.get(Servo.class, "intakeLift");
        intakeGate = hardwareMap.get(Servo.class, "intakeGate");
        hookLeft = hardwareMap.get(Servo.class, "hookLeft");
        hookRight = hardwareMap.get(Servo.class, "hookRight");

        liftSwitch = hardwareMap.get(DigitalChannel.class, "liftSwitch");
        extensionSwitch = hardwareMap.get(DigitalChannel.class, "extensionSwitch");
        intakeLiftSwitch = hardwareMap.get(DigitalChannel.class, "intakeLiftSwitch");
        beamA = hardwareMap.get(DigitalChannel.class, "beamB");
        beamB = hardwareMap.get(DigitalChannel.class, "beamA");
        purpleServo = hardwareMap.get(Servo.class, "purpleServo");

        grabPurple();
        closeIntakeGate();
        basketIn();
        intakeLiftUp();
        basketRotate.setPosition(0.25);
        rotateTarget = 0.025;
//        hooksDown();
        releaseA();
    }

    public void drive(double pow, double heading, double turn) {
        //right strafe they all turn in left strafe they all turn out
        heading-=Math.PI/4.0;
        pow*=Math.sqrt(2);
        double pow1=pow*Math.cos(heading);
        double pow2=pow*Math.sin(heading);
        frontleft.setPower(pow1-turn);
        backleft.setPower((pow2-turn));
        frontright.setPower(pow2+turn);
        backright.setPower((pow1+turn));
    }

    public void intakeLiftUp() {
        intakeLift.setPosition(1);
    }

    public void intakeLiftDown() {
        intakeLift.setPosition(0.001);
    }

    public void openIntakeGate() {
        intakeGate.setPosition(0.4);
    }

    public void closeIntakeGate() {
        intakeGate.setPosition(0.95);
    }

    public boolean isIntakeUp() {
        return intakeLiftSwitch.getState();
    }

    public void intakeOn() {
        intake.setPower(-1);
    }

    public void intakeReverse() {
        intake.setPower(1);
    }

    public void intakeSoftReverse() {
        intake.setPower(0.285);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void intakeTransfer() {
//        openIntakeGate();
        intake.setPower(-0.48);
    }

    public void basketOutMucho() {
        basketPivot.setPosition(0.178);
    }

    public void basketOut() {
        basketPivot.setPosition(0.2);
    }

    public void basketOutLess() {
        basketPivot.setPosition(0.217);
    }

    public void basketIn() {
        basketPivot.setPosition(0.533);
    }

    public void basketOutUpMore() {basketPivot.setPosition(0.212);}

    public void rotateBasket() {
        if (rotateTarget == 0.69) {
            rotateTarget = 0.025;
            basketRotate.setPosition(0.025);
        } else {
            rotateTarget = 0.69;
            basketRotate.setPosition(0.69);
        }
    }

    public void angleBasketRight() {
        rotateTarget = 0.912;
        basketRotate.setPosition(0.912);
    }

    public void angleBasketLeft() {
        rotateTarget = 0.468;
        basketRotate.setPosition(0.468);
    }

    public boolean liftDown() {
        return !liftSwitch.getState();
    }

    public boolean extensionIn() {
        return !extensionSwitch.getState();
    }

    public void releaseA() {
        pixelA.setPosition(0.835);
    }

    public void clampA() {
        pixelA.setPosition(0.68);
    }

    public void releaseB() {
        pixelB.setPosition(0.92);
    }

    public void clampB() {
        pixelB.setPosition(0.77);
    }

    public void clampDrone() {
        drone.setPosition(0.2);
    }

    public void releaseDrone() {
        drone.setPosition(0.35);
    }

    public boolean beamABroken() {
        return !beamA.getState();
    }
    public boolean beamBBroken() {
        return !beamB.getState();
    }

    public void hooksUp() {
        hookLeft.setPosition(0.17);
        hookRight.setPosition(0.9);
    }

    public void hooksDown() {
        hookLeft.setPosition(0.5);
        hookRight.setPosition(0.5);
    }

    public void hookDrone() {
        hookRight.setPosition(0.61);
    }
    public void hooksDownAfterDrone() {
        hookRight.setPosition(0.49);
    }

    public void intakeHeight2() {
        intakeLift.setPosition(0.07);
    }

    public void intakeHeight3() {
        intakeLift.setPosition(0.2);
    }

    public void intakeHeight4() {
        intakeLift.setPosition(0.28);
    }

    public void intakeHeight5() {
        intakeLift.setPosition(0.39);
    }

    public void releasePurple() {
        purpleServo.setPosition(1);
    }

    public void grabPurple() {
        purpleServo.setPosition(0.68);
    }

    public void angleBasketMax() {
        basketPivot.setPosition(0.17);
    }


}