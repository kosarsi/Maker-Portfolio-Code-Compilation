package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "NEW Slides RUN")

public class NEWSlidesRun extends LinearOpMode {

    //Hardware map
    SlidesHardwareControl kraken=new SlidesHardwareControl();

    //Sensors
    DigitalChannel clawLimit;
    DistanceSensor clawSensor;
    DigitalChannel intakeSlidesIn;
    DigitalChannel outtakeSlidesIn;
    ColorRangeSensor line;

    //Autoglide variables
    String state = "not Running";
    double grabDistance = 7.5;
    int cycles = 0;
    double currentTime = 10000;
    boolean shouldAuto = false;
    boolean newTrigger = true;
    //Autoglide timer
    ElapsedTime liftTimer=new ElapsedTime();
    ElapsedTime errorTimeout=new ElapsedTime();
    double startTime=100000;

    boolean toScore = false;
    boolean readyDown = false;
    boolean normalUpControl = true;
    boolean readyFlipIn = false;
    double timeDelay=100000;
    String transferState = "ready";
    double transferDelay = 100000;
    boolean trackpadRetract = true;
    boolean clawOpening = false;
    boolean mid = false;

    boolean outIn = true;
    boolean inIn = true;
    boolean outExtending = false;
    boolean inExtending = false;
    boolean outOut = false;
    boolean inOut = false;
    double clawDelay = 10000;
    boolean inRetracting = false;
    boolean outRetracting = false;
    boolean transfered = true;
    double transferDelayAuto = 10000;
    double retractError = 10000;
    double cycleTime = 10000;
    double cycleStartTime = 10000;
    boolean clawClosing = false;
    double outError = 10000;


    //Drive variables
    double offset= Math.PI/2, heading, powGoal=0, turn, x, y, pow1, pow2;
    //Drive angle
    Orientation angles;

    public void runOpMode() throws InterruptedException {
        //init hardwareMap
        kraken.initTele(hardwareMap);

        //drive variables
        double turnMultiplier = 1;
        double driveMultiplier = 1;
        Boolean driverControl = true;

        //IMU Setup
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //reset intake encoders
        kraken.trussRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kraken.trussLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Map sensors
        clawLimit = hardwareMap.get(DigitalChannel.class, "clawLimit");
        clawSensor = hardwareMap.get(DistanceSensor.class, "clawSensor");
        outtakeSlidesIn = hardwareMap.get(DigitalChannel.class, "outDownLimit");
        intakeSlidesIn = hardwareMap.get(DigitalChannel.class, "slidesLimit");
        line = hardwareMap.get(ColorRangeSensor.class, "lineSensor");

        //Set ready
        telemetry.addData("Ready", "");
        telemetry.update();

        waitForStart();
        kraken.flipIntake();

        //update angle
        offset = Math.toRadians(180)+kraken.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        while (opModeIsActive()) {
            //get drive input data
            angles = kraken.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            x = Math.pow(gamepad1.left_stick_x,3);
            y = Math.pow(gamepad1.left_stick_y,3);
            turn = Math.pow(gamepad1.right_stick_x, 3);
            if (gamepad1.left_bumper){
                turn = gamepad1.right_stick_x / 3;
            }
            else {
                turn = Math.pow(gamepad1.right_stick_x, 3);
            }

            //Drive Controls
            if (driverControl){
                //Activate Cubic Drive
                if (gamepad1.left_stick_button) {
                    if (x == 0 && y == 0) {
                        heading = 0;
                    } else {
                        heading = -Math.atan(y / x) + (x >= 0 ? Math.PI : 0);
                        heading -= (angles.firstAngle - offset);
                        x = Math.pow(gamepad1.left_stick_x, 3);
                        y = Math.pow(gamepad1.left_stick_y, 3);
                        powGoal = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
                    }
                } else{
                    if (x==0&&y==0) heading=0;
                    else heading=-Math.atan(y/x)+(x>=0?Math.PI:0);
                    heading-=(angles.firstAngle-offset);
                    powGoal=Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
                }
                //Dpad controls (Not Cartesian)
                if(gamepad1.dpad_up)kraken.drive(-.8,-Math.PI/2,0);
                else if(gamepad1.dpad_down)kraken.drive(-.8,Math.PI/2,0);
                else if (gamepad1.dpad_right) kraken.drive(-.8, Math.PI, 0);
                else if (gamepad1.dpad_left) kraken.drive(-.8, 0, 0);
                else kraken.drive(driveMultiplier * powGoal, heading + Math.PI, -turnMultiplier * turn);
            }

            //reset gyro
            if (gamepad1.y) {
                offset = Math.toRadians(180)+kraken.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            }

            //not in autoglide
            if (!shouldAuto) {
                //basket up
                if (gamepad2.circle) {
                    kraken.clawTransfer();
                    kraken.flipScore();
                }
                //basket down
                if (gamepad2.square) {
                    kraken.flipIntake();
                }
                //claw open
                if (gamepad2.right_bumper) {
                    kraken.clawOpen();
                }
                //claw close
                if (gamepad2.left_bumper) {
                    kraken.clawClose();
                }
                //center outtake slides
                if (gamepad2.right_stick_button) {
                    kraken.turnCenter();
                }
                //pivot outtake slides left
                if (gamepad2.right_stick_x < -0.5) {
                    kraken.turnLeft();
                }
                //pivot outtake slides right
                if (gamepad2.right_stick_x > 0.5) {
                    kraken.turnRight();
                }
                //flip claw down
                if (gamepad2.triangle){
                    kraken.clawOutBeacon();
                }
                if (gamepad2.left_stick_y>0.75 && transferState == "ready") {
                    kraken.clawOut();
                    kraken.flipArmUp();
                }
                //flip claw up
                if (gamepad2.left_stick_y<-0.75 && transferState == "ready") {
                    kraken.clawIn();
                    kraken.flipArmUp();
                }
                //flip claw down and open
                if (gamepad2.left_stick_x>0.75 && transferState == "ready"){
                    kraken.clawOut();
                    kraken.clawOpen();
                    kraken.flipArmUp();
                }
                if (gamepad2.left_stick_x<-0.75 && transferState == "ready"){
                    kraken.clawLowJunctipn();
                    kraken.flipArmUp();
                }
                if (gamepad2.dpad_right){
                    kraken.clawOutMid();
                }
                if (gamepad2.dpad_left){
                    kraken.clawOutTop();
                }
                if (gamepad2.cross){
                    kraken.clawFlipLevel();
                }
                //reset intake slides encoder
                if (gamepad2.options){
                    kraken.slidesReset();
                }
                //retract intake slides
                else if (gamepad1.cross){
                    kraken.intakeIn();
                }
                //set slide power to stick
                else if (gamepad2.right_trigger>0 && transferState == "ready"){
                    kraken.slides(-gamepad2.right_trigger);
                    trackpadRetract = true;
                }
                else if (gamepad2.left_trigger>0 && transferState == "ready" && trackpadRetract){
                    kraken.slides(gamepad2.left_trigger);
                }
                else if (!intakeSlidesIn.getState() && transferState == "ready"){
                    kraken.holdIntakeTele();
                    trackpadRetract = false;
                }
                else if (intakeSlidesIn.getState() && transferState == "ready"){
                    trackpadRetract = true;
                    kraken.slides(0);
                }
                //outtake auto trigger high
                if (gamepad1.square && (normalUpControl || toScore)){
                    kraken.clawOpen();
                    //kraken.clawTransfer();
                    //kraken.outUp(30);
                    kraken.vslamUp();
                    toScore = true;
                    readyDown = false;
                    normalUpControl = false;
                    clawOpening = true;
                    timeDelay = liftTimer.time();
                    mid = false;
                }
                //outtake auto trigger mid
                if (gamepad1.circle && (normalUpControl || toScore)){
                    kraken.clawOpen();
                    //kraken.clawTransfer();
                    //kraken.outUp(450);
                    kraken.vslamUp();
                    toScore = true;
                    readyDown = false;
                    normalUpControl = false;
                    clawOpening = true;
                    mid = true;
                    timeDelay = liftTimer.time();
                }
                //set outtake slide power to stick
                else if (normalUpControl && (gamepad2.right_stick_y>0.5 || gamepad2.right_stick_y<-0.5)){
                    kraken.upSlides(gamepad2.right_stick_y);
                }
                //vslam up
                if (gamepad2.dpad_up){
                    kraken.vslamUp();
                }
                //vslam down
                else if(gamepad2.dpad_down){
                    kraken.vslamDown();
                }
                //auto outtake run
                if (!normalUpControl && toScore && clawOpening && (liftTimer.time()-timeDelay)>0.5){
                    if (mid){
                        kraken.outUp(430);
                    }
                    else {
                        kraken.outUp(-10);
                    }
                    kraken.clawMid();
                    kraken.flipHalf();
                    clawOpening = false;
                }
                if (!normalUpControl && toScore && !clawOpening && gamepad1.right_trigger>0.5){
                    kraken.flipScore();
                    toScore = false;
                    readyFlipIn = true;
                    timeDelay = liftTimer.time();
                }
                if (!normalUpControl && readyFlipIn && (liftTimer.time()-timeDelay)>0.45){
                    kraken.flipIntake();
                    kraken.vslamDown();
                    readyDown = true;
                    readyFlipIn = false;
                    kraken.outDown();
                    kraken.upSlides(0.5);
                    timeDelay = liftTimer.time();
                }
                if (!normalUpControl && readyDown && (liftTimer.time()-timeDelay)>0.8){// && !outtakeSlidesIn.getState()
                    kraken.upSlides(0);
                    readyDown = false;
                    normalUpControl = true;
                }
                //end auto outtake run

                //auto intake trigger
                if (false){
                    kraken.clawClose();
                    transferState = "closing";
                    transferDelay = liftTimer.time();
                }
                //auto intake run
                if ((transferState == "closing") && (liftTimer.time()-transferDelay)>0.45){
                    kraken.clawIn();
                    kraken.slides(0.75);
                    transferState = "flipping";
                    transferDelay = liftTimer.time();
                }
                if ((transferState == "flipping") && (!clawLimit.getState() || liftTimer.time()-transferDelay>1) && !intakeSlidesIn.getState()){
                    //kraken.clawOpen();
                    kraken.slides(0.05);
                    transferState = "ready";
                }
                if (gamepad1.share){
                    transferState = "ready";
                    normalUpControl = true;

                    outIn = true;
                    inIn = true;
                    outExtending = false;
                    inExtending = false;
                    outOut = false;
                    inOut = false;
                    clawDelay = 10000;
                    inRetracting = false;
                    outRetracting = false;
                    transfered = true;
                    transferDelayAuto = 10000;
                    retractError = 10000;
                    cycleTime = 10000;
                    cycleStartTime = 10000;
                    outError = 10000;
                }
                //end auto intake run

                if (gamepad2.options){
                    kraken.flipArmUp();
                }
                else if (gamepad2.share || gamepad2.left_stick_button){
                    kraken.flipArmDown();
                }

            } //end of normal control



            //Autoglide trigger
            if (gamepad1.left_trigger > 0.4){
                shouldAuto = true;
                //kraken.clawOpen();
                //kraken.clawOutLow();
                //driverControl = false;
                if (!gamepad1.right_bumper){
                    kraken.lockWheels();
                }
                if (newTrigger){
                    state = "start";
                    currentTime = liftTimer.time();
                    newTrigger = false;
                    cycleStartTime = errorTimeout.time();
                    kraken.clawOpen();

                    outIn = true;
                    inIn = true;
                    outExtending = false;
                    inExtending = false;
                    outOut = false;
                    inOut = false;
                    clawDelay = 10000;
                    inRetracting = false;
                    outRetracting = false;
                    transfered = true;
                    transferDelayAuto = 10000;
                    retractError = 10000;
                    cycleTime = 10000;
                    cycleStartTime = 10000;
                    outError = 10000;
                }
            }
            else {
                shouldAuto = false;
                newTrigger = true;
                driverControl = true;
                state = "not running";
            }

            //autoglide

            //V6
            if (shouldAuto){
                if (transfered && inIn && outIn && !inExtending && !outExtending){
                    kraken.clawOut();
                    kraken.slides(-0.45);
                    kraken.upSlides(-1);
                    //kraken.outUp(10);
                    //kraken.vslamUp();
                    inIn = false;
                    outIn = false;
                    transfered = false;
                    inExtending = true;
                    outExtending = true;
                }
                if (outExtending && (kraken.up.getCurrentPosition()<-400)){
                    kraken.flipScore();
                }
                if (outExtending && (kraken.up.getCurrentPosition()<-400)){
                    kraken.vslamUp();
                }
                if (outExtending && (kraken.up.getCurrentPosition()<-625)){ //!kraken.up.isBusy() ||
                    kraken.outUp(-10);
                    outExtending = false;
                    outOut = true;
                    kraken.flipScore();
                    cycleTime = errorTimeout.time()-cycleStartTime;
                    cycleStartTime = errorTimeout.time();
                    cycles++;
                    timeDelay = errorTimeout.time();
                }
                if (outOut && errorTimeout.time()-timeDelay>0.3){
                    if (errorTimeout.time()-timeDelay>0.05){
                        kraken.vslamDown();
                    }
                    kraken.flipIntake();
                    kraken.outDown();
                    outError = errorTimeout.time();
                    outOut = false;
                    outRetracting = true;
                    kraken.vslamDown();
                }
                if (outRetracting && (!outtakeSlidesIn.getState() || errorTimeout.time()-outError>1)){
                    outRetracting = false;
                    outIn = true;
                }
                if (inExtending && ((clawSensor.getDistance(DistanceUnit.INCH)<17) || (line.getLightDetected()>0.2) || (line.getDistance(DistanceUnit.INCH)<6))){
                    kraken.slides(0);
                    clawDelay = errorTimeout.time();
                    kraken.clawClose();
                    clawClosing = true;
                    inExtending = false;
                    inOut = true;
                }
                if (inOut && errorTimeout.time()-clawDelay>0.5){
                    kraken.slides(0.5);
                    kraken.clawIn();
                }
                if (outIn && inOut && errorTimeout.time()-clawDelay>0.5){
                    kraken.slides(0.95);
                    inOut = false;
                    inRetracting = true;
                    retractError = errorTimeout.time();
                }
                if (inRetracting && ((!intakeSlidesIn.getState() && !clawLimit.getState()) || errorTimeout.time()-retractError>1) && errorTimeout.time()-retractError>0.15){
                    kraken.slides(0.25);
                    kraken.clawOpen();
                    inRetracting = false;
                    inIn = true;
                    transferDelayAuto = errorTimeout.time();
                }
                if (inIn && errorTimeout.time()-transferDelayAuto>0.55){
                    transfered = true;
                }
            }

            telemetry.addLine("Version: 2.0");
            telemetry.addLine("Running");
            telemetry.addData("Claw Distance", clawSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Line Sensor", line.getLightDetected());
            telemetry.addData("line sensor distance", line.getDistance(DistanceUnit.INCH));
            telemetry.addData("line state", line.getDistance(DistanceUnit.INCH)<6);
            telemetry.addData("Claw State", clawLimit.getState());
            telemetry.addData("Intake State", intakeSlidesIn.getState());
            telemetry.addData("Out State", outtakeSlidesIn.getState());
            telemetry.addData("","");
            telemetry.addData("Autoglide State", shouldAuto);
            telemetry.addData("Autoglide Cycles", cycles);
            telemetry.addData("Autoglide Cycle Time", cycleTime);
            telemetry.update();
        }
    }
}