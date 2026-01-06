package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.FarRedPipeline;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "Far Red (2+1)")
public class RedLeftCycle extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.0508;

        int ID_TAG_OF_INTEREST = 5; // Tag ID 5 from the 36h11 family

        AprilTagDetection tagOfInterest = null;

        AnalogInput ultrasonic0 = hardwareMap.get(AnalogInput.class, "ultrasonic0");
        AnalogInput ultrasonic1 = hardwareMap.get(AnalogInput.class, "ultrasonic1");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37, -61, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        HardwareControl kraken = new HardwareControl();
        kraken.initAuto(hardwareMap);

        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        FarRedPipeline farRedPipeline = new FarRedPipeline();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameramonitorviewid", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        camera.setPipeline(farRedPipeline);

        ElapsedTime timer = new ElapsedTime();

        kraken.releaseA();
        kraken.releaseB();

        Trajectory centerOne = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-38, -15, Math.toRadians(140)))
                .build();
        TrajectorySequence centerTwo = drive.trajectorySequenceBuilder(centerOne.end())
                .lineTo(new Vector2d(-42, -11))
                .lineToLinearHeading(new Pose2d(-50, -11, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(90), 14.5),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        TrajectorySequence centerThree = drive.trajectorySequenceBuilder(centerTwo.end())
                .lineTo(new Vector2d(34, -11))
                .splineToConstantHeading(new Vector2d(40, -35), Math.toRadians(-70))
                .build();

        Trajectory rightOne = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-38, -50))
                .splineToSplineHeading(new Pose2d(-32, -32, Math.toRadians(195)), Math.toRadians(30))
                .build();
        Trajectory rightTwo = drive.trajectoryBuilder(rightOne.end())
                .lineToLinearHeading(new Pose2d(-50, -11, Math.toRadians(180)))
                .build();
        Trajectory rightThree = drive.trajectoryBuilder(rightTwo.end())
                .lineTo(new Vector2d(32, -11))
                .splineToConstantHeading(new Vector2d(40, -35), Math.toRadians(-60))
                .build();

        Trajectory leftOne = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-46, -25, Math.toRadians(110)))
                .build();
        TrajectorySequence leftTwo = drive.trajectorySequenceBuilder(leftOne.end())
                .lineTo(new Vector2d(-46, -20))
                .splineToLinearHeading(new Pose2d(-50, -11, Math.toRadians(180)), Math.toRadians(120), SampleMecanumDrive.getVelocityConstraint(30, Math.toRadians(90), 14.5),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .build();
        TrajectorySequence leftThree = drive.trajectorySequenceBuilder(leftTwo.end())
                .lineTo(new Vector2d(32, -11))
                .splineToConstantHeading(new Vector2d(40, -35), Math.toRadians(-60))
                .build();

        boolean shouldMove = true;
        while (opModeInInit()) {
            telemetry.addData("Team Prop Location", farRedPipeline.getDetection());
            telemetry.update();
            if(shouldMove) {
                kraken.extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                kraken.extension.setPower(0.75);
            }
            if (kraken.extensionIn()) {
                kraken.extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                kraken.extension.setPower(0);
                shouldMove = false;
            }
        }

        waitForStart();

        kraken.releaseA();
        kraken.releaseB();
        kraken.extension.setPower(0.7);
        kraken.extension.setTargetPosition(0);
        kraken.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        String detection = farRedPipeline.getDetection();
        camera.setPipeline(aprilTagDetectionPipeline);

        if (detection.equals("middle")) {
            drive.followTrajectory(centerOne);
            kraken.releasePurple();
            drive.followTrajectorySequence(centerTwo);

            kraken.closeIntakeGate();
            kraken.intakeOn();
            kraken.extension.setTargetPosition(-325);
            while (kraken.extension.getCurrentPosition() > -315) {

            }
            kraken.intakeOn();
            kraken.intakeHeight5();
            sleep(800);
            kraken.intakeLiftUp();
            sleep(400);
            kraken.intakeHeight5();
            sleep(800);
            kraken.intakeOff();
            kraken.intakeLiftUp();
            timer.reset();
            while (!kraken.isIntakeUp() && timer.seconds() < 1.5) {

            }
            kraken.extension.setTargetPosition(5);
            while (true) {
                if (kraken.extensionIn() && kraken.extension.getCurrentPosition() > -10) {
                    break;
                }
            }
            kraken.openIntakeGate();
            kraken.intakeTransfer();
            timer.reset();
            while (timer.seconds() < 2.5) {
                if (timer.seconds() > 1.5) {
                    kraken.intakeOn();
                } else if (timer.seconds() > 1) {
                    kraken.intakeOff();
                } else {
                    kraken.intakeTransfer();
                }
                if (kraken.beamABroken()) {
                    kraken.clampA();
                    if (kraken.beamBBroken()) {
                        kraken.clampB();
                        break;
                    }
                }
            }
            kraken.clampA();
            kraken.clampB();
            kraken.intakeOff();

            drive.followTrajectorySequence(centerThree);
            kraken.lift.setPower(0.75);
            kraken.lift.setTargetPosition(-250);
            kraken.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            kraken.basketOut();

            double x = 0;
            double z = 0;

            boolean detected = false;
            while (!detected) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if (currentDetections.size() > 0) {
                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == 5) {
                            x = tag.pose.x * FEET_PER_METER * 12;
                            z = tag.pose.z * FEET_PER_METER * 12;
                            telemetry.addData("X", x);
                            telemetry.addData("Z", z);
                            telemetry.update();
                            detected = true;
                        }
                    }
                }
            }

            telemetry.update();

            double ultrasonicYEstimate = -1 * ((ultrasonic0.getVoltage() * 80 - 9.8) - 35.5) * Math.cos(drive.getPoseEstimate().getHeading());
            double ultrasonicXEstimate = -1 * ((ultrasonic1.getVoltage() * 80 - 9.8) - 35.5) * Math.cos(drive.getPoseEstimate().getHeading());
            Pose2d newPose = new Pose2d((61 - z - 6.875) * 0.7 + ultrasonicXEstimate * 0.3, (-35.5 + x) * 0.7 + ultrasonicXEstimate * 0.3, Math.toRadians(180));
            drive.setPoseEstimate(newPose);

            Trajectory centerFive = drive.trajectoryBuilder(newPose)
                    .lineToLinearHeading(new Pose2d(53, -35, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(180), 14.5),
                            SampleMecanumDrive.getAccelerationConstraint(20))
                    .build();
            Trajectory centerSix = drive.trajectoryBuilder(centerFive.end())
                    .lineToLinearHeading(new Pose2d(45, -12, Math.toRadians(90)))
                    .build();

            kraken.rotateBasket();
            drive.followTrajectory(centerFive);
            kraken.releaseA();
            sleep(400);
            kraken.lift.setTargetPosition(-380);
            kraken.basketOut();
            sleep(500);
            kraken.angleBasketLeft();
            sleep(500);
            kraken.releaseB();
            sleep(400);
            kraken.rotateBasket();
            kraken.basketIn();
            sleep(250);

            kraken.lift.setTargetPosition(10);
            timer.reset();
            while (!kraken.liftDown() && timer.seconds() < 1) {
            }
            kraken.lift.setPower(0);
            kraken.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.followTrajectory(centerSix);

        } else if (detection.equals("right")) {
            drive.followTrajectory(rightOne);
            kraken.releasePurple();
            drive.followTrajectory(rightTwo);
            kraken.closeIntakeGate();
            kraken.intakeOn();
            kraken.extension.setTargetPosition(-300);
            while (kraken.extension.getCurrentPosition() > -290) {

            }
            kraken.intakeOn();
            kraken.intakeHeight5();
            sleep(800);
            kraken.intakeLiftUp();
            sleep(400);
            kraken.intakeHeight5();
            sleep(800);
            kraken.intakeOff();
            kraken.intakeLiftUp();
            timer.reset();
            while (!kraken.isIntakeUp() && timer.seconds() < 1.5) {

            }
            kraken.extension.setTargetPosition(5);
            while (true) {
                if (kraken.extensionIn() && kraken.extension.getCurrentPosition() > -10) {
                    break;
                }
            }
            kraken.openIntakeGate();
            kraken.intakeTransfer();
            timer.reset();
            while (timer.seconds() < 2.5) {
                if (timer.seconds() > 1.5) {
                    kraken.intakeOn();
                } else if (timer.seconds() > 1) {
                    kraken.intakeOff();
                } else {
                    kraken.intakeTransfer();
                }
                if (kraken.beamABroken()) {
                    kraken.clampA();
                    if (kraken.beamBBroken()) {
                        kraken.clampB();
                        break;
                    }
                }
            }
            kraken.clampA();
            kraken.clampB();
            kraken.intakeOff();

            drive.followTrajectory(rightThree);
            kraken.lift.setPower(0.75);
            kraken.lift.setTargetPosition(-220);
            kraken.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            kraken.angleBasketMax();
            double x = 0;
            double z = 0;

            boolean detected = false;
            while (!detected) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if (currentDetections.size() > 0) {
                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == 5) {
                            x = tag.pose.x * FEET_PER_METER * 12;
                            z = tag.pose.z * FEET_PER_METER * 12;
                            telemetry.addData("X", x);
                            telemetry.addData("Z", z);
                            telemetry.update();
                            detected = true;
                        }
                    }
                }
            }

            telemetry.update();
            double ultrasonicYEstimate = -1 * ((ultrasonic0.getVoltage() * 80 - 9.8) - 35.5) * Math.cos(drive.getPoseEstimate().getHeading());
            double ultrasonicXEstimate = -1 * ((ultrasonic1.getVoltage() * 80 - 9.8) - 35.5) * Math.cos(drive.getPoseEstimate().getHeading());
            Pose2d newPose = new Pose2d((61 - z - 6.875) * 0.7 + ultrasonicXEstimate * 0.3, (-35.5 + x) * 0.7 + ultrasonicXEstimate * 0.3, Math.toRadians(180));
            drive.setPoseEstimate(newPose);

            Trajectory rightFive = drive.trajectoryBuilder(newPose)
                    .lineToLinearHeading(new Pose2d(53, -41.5, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(180), 14.5),
                            SampleMecanumDrive.getAccelerationConstraint(20))
                    .build();
            Trajectory rightSix = drive.trajectoryBuilder(rightFive.end())
                    .lineToLinearHeading(new Pose2d(45, -12, Math.toRadians(90)))
                    .build();

            kraken.rotateBasket();
            drive.followTrajectory(rightFive);
            kraken.releaseA();
            sleep(400);
            kraken.lift.setTargetPosition(-315);
            sleep(400);
            kraken.basketOut();
            sleep(400);
            kraken.angleBasketLeft();
            sleep(400);
            kraken.releaseB();
            sleep(400);
            kraken.lift.setTargetPosition(-400);
            sleep(700);
            kraken.rotateBasket();
            kraken.basketIn();
            sleep(400);
            kraken.lift.setTargetPosition(10);
            timer.reset();
            while (!kraken.liftDown() && timer.seconds() < 1) {
            }
            kraken.lift.setPower(0);
            kraken.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.followTrajectory(rightSix);

        } else {
            drive.followTrajectory(leftOne);
            kraken.releasePurple();
            drive.followTrajectorySequence(leftTwo);
            kraken.intakeOn();
            kraken.closeIntakeGate();
            kraken.extension.setTargetPosition(-400);
            while (kraken.extension.getCurrentPosition() > -390) {

            }
            kraken.intakeOn();
            kraken.intakeHeight5();
            sleep(800);
            kraken.intakeLiftUp();
            sleep(400);
            kraken.intakeHeight5();
            sleep(800);
            kraken.intakeOff();
            kraken.intakeLiftUp();
            timer.reset();
            while (!kraken.isIntakeUp() && timer.seconds() < 1.5) {

            }
            kraken.extension.setTargetPosition(5);
            while (true) {
                if (kraken.extensionIn() && kraken.extension.getCurrentPosition() > -10) {
                    break;
                }
            }
            kraken.openIntakeGate();
            kraken.intakeTransfer();
            timer.reset();
            while (timer.seconds() < 2.5) {
                if (timer.seconds() > 1.5) {
                    kraken.intakeOn();
                } else if (timer.seconds() > 1) {
                    kraken.intakeOff();
                } else {
                    kraken.intakeTransfer();
                }
                if (kraken.beamABroken()) {
                    kraken.clampA();
                    if (kraken.beamBBroken()) {
                        kraken.clampB();
                        break;
                    }
                }
            }
            kraken.clampA();
            kraken.clampB();
            kraken.intakeOff();

            drive.followTrajectorySequence(leftThree);
            kraken.lift.setPower(0.75);
            kraken.lift.setTargetPosition(-220);
            kraken.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);
            kraken.angleBasketMax();
            double x = 0;
            double z = 0;

            boolean detected = false;
            while (!detected) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if (currentDetections.size() > 0) {
                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == 5) {
                            x = tag.pose.x * FEET_PER_METER * 12;
                            z = tag.pose.z * FEET_PER_METER * 12;
                            telemetry.addData("X", x);
                            telemetry.addData("Z", z);
                            telemetry.update();
                            detected = true;
                        }
                    }
                }
            }

            telemetry.update();
            double ultrasonicYEstimate = -1 * ((ultrasonic0.getVoltage() * 80 - 9.8) - 35.5) * Math.cos(drive.getPoseEstimate().getHeading());
            double ultrasonicXEstimate = -1 * ((ultrasonic1.getVoltage() * 80 - 9.8) - 35.5) * Math.cos(drive.getPoseEstimate().getHeading());
            Pose2d newPose = new Pose2d((61 - z - 6.875) * 0.7 + ultrasonicXEstimate * 0.3, (-35.5 + x) * 0.7 + ultrasonicXEstimate * 0.3, Math.toRadians(180));
            drive.setPoseEstimate(newPose);

            Trajectory leftFive = drive.trajectoryBuilder(newPose)
                    .lineToLinearHeading(new Pose2d(53, -28, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(20, Math.toRadians(180), 14.5),
                            SampleMecanumDrive.getAccelerationConstraint(20))
                    .build();
            Trajectory leftSix = drive.trajectoryBuilder(leftFive.end())
                    .lineToLinearHeading(new Pose2d(45, -12, Math.toRadians(90)))
                    .build();

            kraken.rotateBasket();
            drive.followTrajectory(leftFive);
            kraken.releaseA();
            sleep(400);
            kraken.lift.setTargetPosition(-305);
            sleep(400);
            kraken.basketOut();
            sleep(400);
            kraken.angleBasketRight();
            kraken.angleBasketMax();
            sleep(400);
            kraken.releaseB();
            sleep(400);
            kraken.lift.setTargetPosition(-400);
            sleep(700);
            kraken.rotateBasket();
            kraken.basketIn();
            sleep(400);
            kraken.lift.setTargetPosition(10);
            timer.reset();
            while (!kraken.liftDown() && timer.seconds() < 1) {
            }
            kraken.lift.setPower(0);
            kraken.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.followTrajectory(leftSix);
        }
    }

}