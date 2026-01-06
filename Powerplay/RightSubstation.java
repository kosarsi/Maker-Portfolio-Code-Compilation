package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="Right Substation")
public class RightSubstation extends LinearOpMode {
    SlidesHardwareControl kraken=new SlidesHardwareControl();

    String park=null;

    double coneDistance = 14; //4.5
    double coneDistanceBack = 14.0; //6.0
    double coneDistanceBottom = 13.0; //5.25
    double preGrabDistance = 11.0; //28
    double grabSpeed = -0.55;
    double preGrabSpeed = -0.9;
    double inSpeed = 0.9;
    double preFlipSpeed = 0.15;
    double slideUpSpeed = -1;
    double startTime;
    boolean newFlip = true;
    boolean flipped = false;

    boolean intakeMoving = false;
    boolean outtakeMoving = false;

    DigitalChannel clawLimit;
    DigitalChannel intakeSlidesIn;
    DigitalChannel outtakeSlidesIn;
    DistanceSensor clawSensor;
    ElapsedTime errorTimeout = new ElapsedTime();
    ColorRangeSensor line;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 4;
    int MIDDLE = 5;
    int RIGHT = 6;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        kraken.initTele(hardwareMap);

        clawLimit = hardwareMap.get(DigitalChannel.class, "clawLimit");
        intakeSlidesIn = hardwareMap.get(DigitalChannel.class, "slidesLimit");
        outtakeSlidesIn = hardwareMap.get(DigitalChannel.class, "outDownLimit");
        clawSensor = hardwareMap.get(DistanceSensor.class, "clawSensor");

        intakeSlidesIn.setMode(DigitalChannel.Mode.INPUT);
        line = hardwareMap.get(ColorRangeSensor.class, "lineSensor");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35.25, 64, Math.toRadians(90));
        //Pose2d shouldBe = new Pose2d(-36, 7, Math.toRadians(175));
        drive.setPoseEstimate(startPose);

        Trajectory initial = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-6.5, 55.5))
                .splineToConstantHeading(new Vector2d(-11.5, 45), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-12.5, 34), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-15, 19, Math.toRadians(187.8)), Math.toRadians(270))
                .build();

        Trajectory parkLeft = drive.trajectoryBuilder(initial.end())
                .splineToLinearHeading(new Pose2d(-13.5, 24, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Trajectory parkMiddle = drive.trajectoryBuilder(initial.end())
                .splineToConstantHeading(new Vector2d(-12.5, 13), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-32, 10), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-36, 25, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Trajectory parkRight = drive.trajectoryBuilder(initial.end())
                .splineToConstantHeading(new Vector2d(-12.5, 14), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-34, 16), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-54, 16.5), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-62, 25, Math.toRadians(90)), Math.toRadians(90))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        kraken.flipIntake();
        kraken.vslamDown();
        kraken.turnStart();
        kraken.clawIn();
        kraken.clawClose();
        kraken.slidesReset();
        sleep(500);
        kraken.up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kraken.resetIntake();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == RIGHT || tag.id == MIDDLE)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (tagOfInterest.id == 5) {
            // Go left
            park = "left";

        } else if (tagOfInterest.id == 4) {
            // Go forward
            park = "middle";

        } else if (tagOfInterest.id == 6) {
            // Go right
            park = "right";
        }

        kraken.holdIntake();
        kraken.turnLeft();

        drive.followTrajectory(initial);

        kraken.lockWheelsRight();

        cycleFAST(1, coneDistance, -10, 325,1.15, 20);
        cycleFAST(2, coneDistanceBack, -10, 275,1.15, preGrabDistance);
        cycleFAST(3, coneDistanceBack, -10, 275,1.15, preGrabDistance);
        cycleFAST(4, coneDistanceBottom, -10, 275,1.15, preGrabDistance);
        cycleFAST(5, coneDistanceBottom, -20, 275,1.15, 10);

        kraken.clawMid();
        kraken.vslamUp();
        kraken.outUp(45);
        startTime = errorTimeout.time();
        while (kraken.up.getCurrentPosition()>-565){
            if (kraken.up.getCurrentPosition()<-400){
                kraken.flipScorePartial();
            }
            if (errorTimeout.time()-startTime>3){
                break;
            }
        }
        kraken.vslamDown();
        sleep(350);
        kraken.flipIntake();
        sleep(50);
        kraken.vslamDown();
        kraken.outDown();
        kraken.clawTransfer();
        kraken.holdIntake();
        kraken.turnCenter();
        kraken.clawClose();

        //parking
        if (park == "left"){
            drive.followTrajectory(parkLeft);
        }
        else if (park == "middle"){
            drive.followTrajectory(parkMiddle);
        }
        else if (park == "right"){
            drive.followTrajectory(parkRight);
        }
        kraken.clawIn();
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    void cycleFAST(double coneNumber, double distanceValue, int offset, int holdTime, double extendTimeout, double startDistance){
        if (coneNumber == 1){
            kraken.clawOutTop();
        }
        else if (coneNumber == 2) {
            kraken.clawOutSecond();
        }
        else if (coneNumber == 3){
            kraken.clawOutMid();
        }
        else if (coneNumber == 4){
            kraken.clawOutLow();
        }
        else if (coneNumber == 5){
            kraken.clawOutBottom6();
        }

        sleep(75);
        kraken.clawOpenMore();
        intakeMoving = true;
        outtakeMoving = true;
        kraken.outUp(offset);
        kraken.slides(preGrabSpeed);
        kraken.vslamUp();
        flipped = false;
        startTime = errorTimeout.time();
        while (outtakeMoving || intakeMoving){
            if (kraken.up.getCurrentPosition()<-350 && outtakeMoving){
                kraken.flipScorePartial();
                flipped = true;
            }
            if (kraken.up.getCurrentPosition()<-610 && outtakeMoving && flipped){
                kraken.upSlides(0);
                kraken.outUp(offset);
                kraken.vslamDown();
                //flipTime = errorTimeout.time();
                outtakeMoving = false;
            }
            if (clawSensor.getDistance(DistanceUnit.INCH) < startDistance || line.getDistance(DistanceUnit.INCH)<6){ //clawSensor.getDistance(DistanceUnit.INCH) < preGrabDistance
                //kraken.trussLeft.getCurrentPosition()<-175
                kraken.slides(0);
                intakeMoving = false;
            }
            if (errorTimeout.time()-startTime>1.5){
                kraken.slides(0);
                kraken.outUp(offset);
                kraken.vslamDown();
                break;
            }
        }
        kraken.slides(0);
        sleep(holdTime);
        kraken.flipIntake();
        kraken.vslamDown();
        kraken.outDown();

        kraken.slides(grabSpeed);
        startTime = errorTimeout.time();
        while (!(line.getDistance(DistanceUnit.INCH)<5.75)){ //5.75
            if (errorTimeout.time()-startTime>extendTimeout){
                break;
            }
        }
        kraken.slides(preFlipSpeed);
        sleep(50);
        kraken.clawClose();
        sleep(450);
        kraken.clawIn();
        sleep(50);
        startTime = errorTimeout.time();
        while (clawLimit.getState() && kraken.trussRight.getCurrentPosition()>-25 && kraken.trussLeft.getCurrentPosition()>-25){
            if (errorTimeout.time()-startTime>2){
                break;
            }
        }
        kraken.slides(inSpeed);
        startTime = errorTimeout.time();
        while (intakeSlidesIn.getState()){
            if (errorTimeout.time()-startTime>3){
                break;
            }
        }
        kraken.slides(0);
        kraken.resetIntake();
        sleep(50);
        kraken.clawOpen();
        sleep(400);
    }

} //end opmode