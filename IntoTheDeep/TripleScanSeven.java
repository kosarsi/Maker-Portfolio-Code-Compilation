package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.units.TimeSpan;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystems.IntakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystems.IntakeLinkage;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystems.IntakeRotate;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystems.IntakeWrist;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystems.OuttakeClaw;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystems.OuttakeRotate;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeSubsystems.OuttakeWrist;

import pedroPathing.constants.FConstantsFast;
import pedroPathing.constants.LConstants;

@Autonomous(name="Triple Scan Seven Sample")
public class TripleScanSeven extends PedroOpMode {

    String currentState = "firstFour";

    private final Pose startPose = new Pose(136.500, 31.000, Math.toRadians(0));
    private final Pose scorePreloadPose = new Pose(120.000, 13.000, Math.toRadians(-26));
    private final Pose scoreOnePose = new Pose(123.000, 9.000, Math.toRadians(-12.5));
    private final Pose grabThreePose = new Pose(125.00, 11.000, Math.toRadians(30));
    private final Pose scoreThreePose = new Pose(120.000, 15, Math.toRadians(-30));
    private final Pose subZeroPose = new Pose(68.000, 53.000, Math.toRadians(-90));
    private final Pose subInterPose = new Pose(85.000, 36.000, Math.toRadians(-30));
    private final Pose subScorePose = new Pose(120, 15, Math.toRadians(-30));

    private Path scorePreload, scoreOne, grabThree, scoreThree, subZero, subTwo, subThree;
    Limelight3A limelight3A;

    public void buildPaths() {

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePreloadPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading());
        scorePreload.setPathEndTimeoutConstraint(1.5);

        scoreOne = new Path(new BezierLine(new Point(scorePreloadPose), new Point(scoreOnePose)));
        scoreOne.setLinearHeadingInterpolation(scorePreloadPose.getHeading(), scoreOnePose.getHeading());

        grabThree = new Path(new BezierLine(new Point(scoreOnePose), new Point(grabThreePose)));
        grabThree.setLinearHeadingInterpolation(scoreOnePose.getHeading(), grabThreePose.getHeading());

        scoreThree = new Path(new BezierLine(new Point(grabThreePose), new Point(scoreThreePose)));
        scoreThree.setLinearHeadingInterpolation(grabThreePose.getHeading(), scoreThreePose.getHeading());

        subZero = new Path(new BezierLine(new Point(scoreThreePose), new Point(subZeroPose)));
        subZero.setLinearHeadingInterpolation(scoreThreePose.getHeading(), subZeroPose.getHeading());

        subTwo = new Path(new BezierLine(new Point(subScorePose), new Point(subZeroPose)));
        subTwo.setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-90));

        subThree = new Path(new BezierLine(new Point(subScorePose), new Point(subZeroPose)));
        subThree.setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-90));

    }

    public TripleScanSeven() {
        super(IntakeClaw.INTAKE_CLAW, IntakeArm.INTAKE_ARM, IntakeWrist.INTAKE_WRIST, IntakeRotate.INTAKE_ROTATE, IntakeLinkage.INTAKE_LINKAGE, Lift.LIFT, OuttakeArm.OUTTAKE_ARM, OuttakeWrist.OUTTAKE_WRIST, OuttakeRotate.OUTTAKE_ROTATE, OuttakeClaw.OUTTAKE_CLAW);
    }

    public Command preloadSample() {
        return new SequentialGroup(
                new ParallelGroup (
                        new FollowPath(scorePreload),
                        AutoCommands.liftPreload(),
                        AutoCommands.extendIntake(-20)
                ),
                AutoCommands.score(0)
        );
    }

    public Command groundOne() {
        return new SequentialGroup(
                new ParallelGroup(
                        AutoCommands.intakeAndRetract(100),
                        AutoCommands.retractLift()
                ),
                new ParallelGroup(
                        new SequentialGroup(
                                AutoCommands.transfer(),
                                new ParallelGroup(
                                        AutoCommands.liftUp(),
                                        new SequentialGroup(
                                                new Delay(TimeSpan.fromMs(200)),
                                                AutoCommands.extendIntake(0)
                                        )
                                )
                        ),
                        new FollowPath(scoreOne)
                ),
                AutoCommands.score(100)
        );
    }

    public Command groundTwo() {
        return new SequentialGroup(
                new ParallelGroup(
                        AutoCommands.intakeAndRetract(100),
                        AutoCommands.retractLift()
                ),
                AutoCommands.transfer(),
                new ParallelGroup(
                        AutoCommands.liftUp(),
                        new SequentialGroup(
                                new Delay(TimeSpan.fromMs(200)),
                                AutoCommands.extendIntake(25)
                        )
                ),
                AutoCommands.score(100)
        );
    }

    public Command groundThree() {
        return new SequentialGroup (
                new ParallelGroup(
                        AutoCommands.retractLift(),
                        new SequentialGroup(
                                new FollowPath(grabThree),
                                AutoCommands.intakeAndRetract(0)
                        )
                ),
                new ParallelGroup(
                        new SequentialGroup(
                                AutoCommands.transfer(),
                                AutoCommands.liftUp()
                        ),
                        new FollowPath(scoreThree)
                ),
                AutoCommands.score(100)
        );
    }

    public Command toSubZero() {
        return new ParallelGroup(
                AutoCommands.retractLift(),
                IntakeLinkage.INTAKE_LINKAGE.linkageSlightlyOut(),
                new FollowPath(subZero)
        );
    }

    SequentialGroup firstFour;

    Command getSubOne;
    Command getSubTwo;
    Command getSubThree;

    ElapsedTime timer;

    boolean getSubOneStarted = false;

    Servo light;

//    Led

    @Override
    public void onInit() {
        Constants.setConstants(FConstantsFast.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight3A.pipelineSwitch(1);
        OuttakeClaw.OUTTAKE_CLAW.close().invoke();
        IntakeClaw.INTAKE_CLAW.open().invoke();
        firstFour = new SequentialGroup(
                preloadSample(),
                groundOne(),
                groundTwo(),
                groundThree(),
                toSubZero()
        );
        timer = new ElapsedTime();
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(0);
    }

    @Override
    public void onStartButtonPressed() {

        firstFour.invoke();
        light.setPosition(0);
        limelight3A.start();

    }

    @Override
    public void onUpdate() {
        if (currentState.equals("firstFour") && firstFour.isDone()) {
            currentState = "scanningFirst";
        }
        if (currentState.equals("scanningFirst")) {
            double[] output = limelight3A.getLatestResult().getPythonOutput();

            // output[1] is the number of detections
            while (output[1] < 1) {
                output = limelight3A.getLatestResult().getPythonOutput();
            }

            Pose subOnePose = new Pose(68 + cameraToPosition(output[2])[0], 53 + cameraToPosition(output[2])[1], Math.toRadians(-90));

            Path grabSubOne = new Path(new BezierLine(new Point(subZeroPose), new Point(subOnePose)));
            grabSubOne.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90));

            PathChain scoreSubOne = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(subOnePose), new Point(subInterPose)))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-30))
                    .setZeroPowerAccelerationMultiplier(10)
                    .addPath(new BezierLine(new Point(subInterPose), new Point(subScorePose)))
                    .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-30))
                    .setPathEndTimeoutConstraint(1.4)
                    .build();

            getSubOne = new SequentialGroup(
                    new ParallelGroup(
                            new FollowPath(grabSubOne),
                            AutoCommands.extendIntake(output[3]-90)
                    ),
                    new ParallelGroup(
                            new SequentialGroup(
                                    // TODO: ADJUST DELAY TO MATCH WHEN THE CLAW FINISHES GRABBING
                                    new Delay(TimeSpan.fromMs(150)),
                                    new FollowPath(scoreSubOne)
                            ),
                            new SequentialGroup(
                                    AutoCommands.intakeAndRetractSub(),
                                    AutoCommands.transfer(),
                                    // TODO: ADJUST DELAY TO MATCH WHEN THE ROBOT ALMOST REACHES THE BUCKET
                                    new Delay(TimeSpan.fromMs(500)),
                                    AutoCommands.liftPreload()
                            )
                    ),
                    AutoCommands.score(0),
                    new ParallelGroup(
                            new FollowPath(subTwo),
                            IntakeLinkage.INTAKE_LINKAGE.linkageSlightlyOut(),
                            AutoCommands.retractLift()
                    )
            );

            getSubOne.invoke();
            currentState = "scoringSubFirst";
            timer.reset();
        }

        if (currentState.equals("scoringSubFirst") && getSubOne.isDone() && timer.seconds() > 3.0) {
            currentState = "scanningSubTwo";
            light.setPosition(0);

            double[] output = limelight3A.getLatestResult().getPythonOutput();

            // output[1] is the number of detections
            while (output[1] < 1) {
                output = limelight3A.getLatestResult().getPythonOutput();
            }

            Pose subTwoPose = new Pose(68 + cameraToPosition(output[2])[0], 53 + cameraToPosition(output[2])[1], Math.toRadians(-90));

            Path grabSubTwo = new Path(new BezierLine(new Point(subZeroPose), new Point(subTwoPose)));
            grabSubTwo.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90));

            PathChain scoreSubTwo = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(subTwoPose), new Point(subInterPose)))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-30))
                    .setZeroPowerAccelerationMultiplier(10)
                    .addPath(new BezierLine(new Point(subInterPose), new Point(subScorePose)))
                    .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-30))
                    .setPathEndTimeoutConstraint(1.4)
                    .build();

            getSubTwo = new SequentialGroup(
                    new ParallelGroup(
                            new FollowPath(grabSubTwo),
                            AutoCommands.extendIntake(output[3]-90)
                    ),
                    new ParallelGroup(
                            new SequentialGroup(
                                    // TODO: ADJUST DELAY TO MATCH WHEN THE CLAW FINISHES GRABBING
                                    new Delay(TimeSpan.fromMs(150)),
                                    new FollowPath(scoreSubTwo)
                            ),
                            new SequentialGroup(
                                    AutoCommands.intakeAndRetractSub(),
                                    AutoCommands.transfer(),
                                    // TODO: ADJUST DELAY TO MATCH WHEN THE ROBOT ALMOST REACHES THE BUCKET
                                    new Delay(TimeSpan.fromMs(500)),
                                    AutoCommands.liftPreload()
                            )
                    ),
                    AutoCommands.score(0),
                    new ParallelGroup(
                            new FollowPath(subThree),
                            IntakeLinkage.INTAKE_LINKAGE.linkageSlightlyOut(),
                            AutoCommands.retractLift()
                    )
            );

            getSubTwo.invoke();
            currentState = "scoringSubSecond";
            timer.reset();
        }

        if (timer.seconds() > 3 && currentState.equals("scoringSubSecond") && getSubTwo.isDone()) {
            currentState = "scanningSubThree";
            light.setPosition(0);


            double[] output = limelight3A.getLatestResult().getPythonOutput();

            // output[1] is the number of detections
            while (output[1] < 1) {
                output = limelight3A.getLatestResult().getPythonOutput();
            }

            Pose subThreePose = new Pose(68 + cameraToPosition(output[2])[0], 53 + cameraToPosition(output[2])[1], Math.toRadians(-90));
            Path grabSubThree = new Path(new BezierLine(new Point(subZeroPose), new Point(subThreePose)));
            grabSubThree.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90));

            PathChain scoreSubThree = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(subThreePose), new Point(subInterPose)))
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-30))
                    .setZeroPowerAccelerationMultiplier(10)
                    .addPath(new BezierLine(new Point(subInterPose), new Point(subScorePose)))
                    .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(-30))
                    .setPathEndTimeoutConstraint(1.6)
                    .build();

            getSubThree = new SequentialGroup(
                    new ParallelGroup(
                            new FollowPath(grabSubThree),
                            AutoCommands.extendIntake(output[3]-90)
                    ),
                    new ParallelGroup(
                            new SequentialGroup(
                                    // TODO: ADJUST DELAY TO MATCH WHEN THE CLAW FINISHES GRABBING
                                    new Delay(TimeSpan.fromMs(150)),
                                    new FollowPath(scoreSubThree)
                            ),
                            new SequentialGroup(
                                    AutoCommands.intakeAndRetractSub(),
                                    AutoCommands.transfer(),
                                    // TODO: ADJUST DELAY TO MATCH WHEN THE ROBOT ALMOST REACHES THE BUCKET
                                    new Delay(TimeSpan.fromMs(500)),
                                    AutoCommands.liftPreload()
                            )
                    ),
                    AutoCommands.score(0),
                    new ParallelGroup(
                            AutoCommands.retractFinal(),
                            IntakeLinkage.INTAKE_LINKAGE.linkageSlightlyOut(),
                            new FollowPath(subThree)
                    )
            );

            getSubThree.invoke();
            currentState = "scoringSubThird";
            timer.reset();

        }

        telemetry.addData("current state", currentState);
        telemetry.update();

    }

    // Returns position of detected sample relative to center of claw
    private static double[] cameraToPosition(double number) {
        // Truncating angle
        double x = Math.floor(number);
        double y = (number - Math.floor(number)) * 1000;
        final double INCHES_PER_PIXEL = 0.025;
        double xOffset = -4.25;
        double yOffset = -5.75;
        double[] output = new double[2];

        output[0] = (x - 320) * INCHES_PER_PIXEL + xOffset;
        output[1] = -(y - 240) * INCHES_PER_PIXEL + yOffset;

        return output;
    }

    @Override
    public void onStop() {
        OuttakeArm.OUTTAKE_ARM.transferPosition().invoke();
    }

}