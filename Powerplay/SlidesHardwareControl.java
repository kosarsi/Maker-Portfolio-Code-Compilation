package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class SlidesHardwareControl {
    DcMotor frontleft, frontright, backleft, backright;
    DcMotor trussRight, trussLeft, up;
    BNO055IMU imu;
    Servo basket, claw, pivotLeft, pivotRight, rotate, vslam, flipArm;
    boolean active = false;
    boolean encoder;
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;
    double xPosition, yPosition;
    double position = 0;
    List<Recognition> updatedRecognitions;
    int recogSize;
    Orientation angles;
    HardwareMap h;
    boolean centerTest, leftTest, rightTest;
    ElapsedTime timer = new ElapsedTime();
    double frontDistance;
    double slidePosition = 0;

    public void initDeadwheel(HardwareMap map) {
        active=true;
        h=map;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu =h.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void initTele(HardwareMap map) {
        active = true;
        h = map;
        frontleft = h.get(DcMotor.class, "frontleft");
        frontright = h.get(DcMotor.class, "frontright");
        backleft = h.get(DcMotor.class, "backleft");
        backright = h.get(DcMotor.class, "backright");

        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);

        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        trussRight = h.get(DcMotor.class, "trussRight");
        trussLeft = h.get(DcMotor.class, "trussLeft");
        trussRight.setDirection(DcMotorSimple.Direction.REVERSE);

        trussRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trussLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        up = h.get(DcMotor.class, "slides");
        up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //up.setDirection(DcMotorSimple.Direction.REVERSE);

        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        basket = h.get(Servo.class, "basket");
        claw = h.get(Servo.class, "claw");
        pivotLeft = h.get(Servo.class, "pivotLeft");
        pivotRight = h.get(Servo.class, "pivotRight");
        rotate = h.get(Servo.class, "rotate");
        vslam = h.get(Servo.class, "vslam");
        flipArm = h.get(Servo.class, "flipArm");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu =h.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }
    public void setEncoder(boolean enc) {
        if (enc) {
            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
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
    public void driveRect(double y, double x, double turn) {
        frontleft.setPower(y+x-turn);
        backleft.setPower((y-x-turn));
        frontright.setPower(y-x+turn);
        backright.setPower((y+x+turn));
    }
    public void driveNormRect(double y, double x, double turn) {
        double flpow=y+x;
        double frpow=y-x;
        double blpow=y-x;
        double brpow=y+x;
        double highestpow = Math.max(Math.max(Math.abs(flpow),Math.abs(frpow)),
                Math.max(Math.abs(blpow),Math.abs(brpow)));

        if (highestpow<1) highestpow=1;
        frontleft.setPower(flpow/highestpow-turn);
        backleft.setPower((blpow/highestpow-turn));
        frontright.setPower(frpow/highestpow+turn);
        backright.setPower((brpow/highestpow+turn));
    }
    public void driveNormStrafe(double pow, double heading, double turn) {
        heading-=Math.PI/4.0;
        pow*=Math.sqrt(2);
        double pow1=pow*Math.cos(heading-getAngle());
        double pow2=pow*Math.sin(heading-getAngle());
        double flpow=pow1-turn;
        double frpow=pow2+turn;
        double blpow=pow2-turn;
        double brpow=pow1+turn;
        frontleft.setPower(flpow);
        backleft.setPower(blpow);
        frontright.setPower(frpow);
        backright.setPower(brpow);
    }
    public void driveNorm(double pow, double heading, double turn) {
        heading-=Math.PI/4.0;
        pow*=Math.sqrt(2);
        double pow1=pow*Math.cos(heading);
        double pow2=pow*Math.sin(heading);
        double flpow=pow1-turn;
        double frpow=pow2+turn;
        double blpow=pow2-turn;
        double brpow=pow1+turn;

        double highestpow = Math.max(Math.max(flpow,frpow), Math.max(blpow,brpow));
        if (highestpow<1) highestpow=1;
        frontleft.setPower(flpow/highestpow);
        backleft.setPower((blpow/highestpow));
        frontright.setPower(frpow/highestpow);
        backright.setPower((brpow/highestpow));
    }
    public double getAngle() {

        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
    public void deactivate(){

    }
    public void resetEncoders(){
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void drive(double pow, double heading) {
        heading-=Math.PI/4.0;
        pow*=Math.sqrt(2);
        double pow1=pow*Math.cos(heading);
        double pow2=pow*Math.sin(heading);
        frontleft.setPower(-pow1);
        backleft.setPower(pow2);
        frontright.setPower(-pow2);
        backright.setPower(pow1);
    }
    public void turn(double turn) {
        frontleft.setPower(-turn);
        backleft.setPower(turn);
        frontright.setPower(turn);
        backright.setPower((-turn));
    }

    public void halt(){
        drive(0,0,0);
    }

    public boolean isActive() {
        return active;
    }

    public void regularDrive(double power) throws InterruptedException {
        backleft.setPower(power);
        backright.setPower(power);
        frontleft.setPower(power);
        frontright.setPower(power);
    }
    public void flipScore(){
        basket.setPosition(1);
    }
    public void flipScorePartial(){
        basket.setPosition(0.95);
    }
    public void flipIntake(){
        basket.setPosition(0);
    }
    public void flipHalf() {
        basket.setPosition(0.28);
    }
    public void clawOpenTransfer(){claw.setPosition(0.285);}
    public void clawOpenMore() {claw.setPosition(0.05);}
    public void clawOpen(){
        claw.setPosition(0.247);
    }
    public void clawClose(){
        claw.setPosition(1);
    }
    public void turnCenter(){
        rotate.setPosition(0.5);
    }
    public void turnStart(){rotate.setPosition(0.42);}
    public void turnRightDifferent(){
        rotate.setPosition(0.41);
    }
    public void turnRight(){
        rotate.setPosition(0.39);
    }
    public void turnLeft(){
        rotate.setPosition(0.59);
    }
    public void slides(double power){
        trussRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trussLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trussRight.setPower(power);
        trussLeft.setPower(power);
    }
    public void slidesReset(){
        trussRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trussLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trussRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trussLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void slidesIn(){
        trussRight.setTargetPosition(0);
        trussLeft.setTargetPosition(0);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussRight.setPower(1);
        trussLeft.setPower(1);
    }
    public void slidesOut(){
        trussRight.setTargetPosition(500);
        trussLeft.setTargetPosition(500);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussRight.setPower(0.75);
        trussLeft.setPower(0.75);
    }
    public void upSlides(double power){
        up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        up.setPower(power);
    }
    public void clawIn(){
        pivotLeft.setPosition(0.00);
        pivotRight.setPosition(1.00);
    }
    public void clawLittleOut(){
        pivotLeft.setPosition(0.05);
        pivotRight.setPosition(0.95);
    }
    public void clawOut(){
        pivotLeft.setPosition(0.95);
        pivotRight.setPosition(0.05);
    }
    public void clawMid(){
        pivotLeft.setPosition(0.2);
        pivotRight.setPosition(0.8);
    }
    public void intakeOut(){
        trussRight.setTargetPosition(-450);
        trussLeft.setTargetPosition(-450);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setPower(1);
        trussRight.setPower(1);
    }
    public void intakeIn(){
        trussRight.setTargetPosition(0);
        trussLeft.setTargetPosition(0);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setPower(1);
        trussRight.setPower(1);
    }
    public void outUp(int modifier){
        up.setTargetPosition(-625+modifier);
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up.setPower(-1);
    }
    public void outDown(){
        up.setTargetPosition(10);
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up.setPower(1);
    }
    public void vslamDown(){
        vslam.setPosition(0);
    }
    public void vslamUp(){
        vslam.setPosition(0.40);
    }
    public void holdIntake(){
        trussRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trussLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trussRight.setPower(0.5);
        trussLeft.setPower(0.5);
    }
    public void releaseIntake(){
        trussRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trussLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trussRight.setPower(0);
        trussLeft.setPower(0);
    }
    public void resetIntake(){
        trussRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trussLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trussRight.setTargetPosition(0);
        trussLeft.setTargetPosition(0);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void intakeOutAuto(){
        trussRight.setTargetPosition(-380);
        trussLeft.setTargetPosition(-380);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setPower(-1);
        trussRight.setPower(-1);
    }
    public void intakeOutAutoMore(int extra){
        trussRight.setTargetPosition(-390-extra);
        trussLeft.setTargetPosition(-390-extra);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setPower(-1);
        trussRight.setPower(-1);
    }
    public void intakeBeforeFlip(){
        trussRight.setTargetPosition(-390);
        trussLeft.setTargetPosition(-390);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setPower(1);
        trussRight.setPower(1);
    }
    public void intakeOutAutoHalf(){
        trussRight.setTargetPosition(-275);
        trussLeft.setTargetPosition(-275);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setPower(-1);
        trussRight.setPower(-1);
    }
    public void intakeOutAutoHalfFar(){
        trussRight.setTargetPosition(-800);
        trussLeft.setTargetPosition(-800);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setPower(-1);
        trussRight.setPower(-1);
        trussRight.setTargetPosition(-800);
        trussLeft.setTargetPosition(-800);
    }
    public void clawOutTop(){
        pivotLeft.setPosition(0.78);
        pivotRight.setPosition(0.22);
    }
    public void clawOutSecond(){
        pivotLeft.setPosition(0.82);
        pivotRight.setPosition(0.18);
    }
    public void clawOutMid(){
        pivotLeft.setPosition(0.86);
        pivotRight.setPosition(0.14);
    }
    public void clawOutLow(){
        pivotLeft.setPosition(0.92);
        pivotRight.setPosition(0.08);
    }
    public void clawOutBottom(){
        pivotLeft.setPosition(0.94);
        pivotRight.setPosition(0.06);
    }
    public void clawOutBeacon(){
        pivotLeft.setPosition(1);
        pivotRight.setPosition(0);
    }
    public void intakeInAuto(){
        trussRight.setTargetPosition(-50);
        trussLeft.setTargetPosition(-50);
        trussLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trussLeft.setPower(1);
        trussRight.setPower(1);
    }
    public double getPosition(){
        position=trussRight.getCurrentPosition();
        return position;
    }
    public void lockWheels(){
        backright.setPower(0.1);
        backleft.setPower(0.17);
        frontright.setPower(-0.05);
        frontleft.setPower(-0.05);
    }
    public void lockWheelsRight(){
        backright.setPower(0.2);
        backleft.setPower(0.21);
        frontright.setPower(-0.1);
        frontleft.setPower(-0.05);
    }

    public void holdIntakeTele(){
        trussRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trussLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        trussRight.setPower(0.15);
        trussLeft.setPower(0.15);
    }
    public void clawTransfer(){
        pivotLeft.setPosition(0.10);
        pivotRight.setPosition(0.90);
    }
    public void clawLowJunctipn(){
        pivotLeft.setPosition(0.45);
        pivotRight.setPosition(0.55);
    }
    public void clawFlipLevel(){
        pivotLeft.setPosition(0.91);
        pivotRight.setPosition(0.09);
    }
    public void flipArmDown(){
        flipArm.setPosition(0.49);
    }
    public void flipArmUp(){
        flipArm.setPosition(0);
    }

    public void clawOutTop6(){
        pivotLeft.setPosition(0.8);
        pivotRight.setPosition(0.2);
    }
    public void clawOutSecond6(){
        pivotLeft.setPosition(0.85);
        pivotRight.setPosition(0.15);
    }
    public void clawOutMid6(){
        pivotLeft.setPosition(0.9);
        pivotRight.setPosition(0.1);
    }
    public void clawOutLow6(){
        pivotLeft.setPosition(0.93);
        pivotRight.setPosition(0.07);
    }
    public void clawOutBottom6(){
        pivotLeft.setPosition(0.97);
        pivotRight.setPosition(0.03);
    }

}