package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="RUN")
public class RUN extends OpMode {



    public enum RobotState {
        READY_POSITION,
        EXTENDING_INTAKE,
        INTAKE_OUT,
        PECKING,
        SAMPLE_GRABBED,
        TRANSFERRING,
        LIFTING_LOW,
        LIFTING_HIGH,
        READY_DROP_BUCKET,
        READY_DROP_LOW,
        DROPPING_BUCKET,
        RETRACTING_LIFT,
        SPECIMEN_INTAKE,
        SPECIMEN_OUT,
        SPECIMEN_SCORE,
        READY_HANG,
        PULLING_SIDE_HOOKS,
        LIFTING_MAIN_HOOKS,
        PULLING_MAIN_HOOKS
    }

    public enum SideHookState {
        DOWN,
        LIFTING,
        UP,
        NEGATIVE_POWER,
        DONE
    }

    ElapsedTime motionProfileTimer = new ElapsedTime();

    boolean high = true;

    HardwareControl kraken;

    double error;
    double power;

    boolean reached = false;

    final double Kp = 0.005;
    final double Kd = 0;
    final double Kf = 0;

    final double k1 = 0.004;
    final double k2 = 0.012;

    boolean intakeIn = false;

    double lastError = 0;
    ElapsedTime PIDTimer;

    int position;

    RobotState robotState;
    SideHookState sideHookState;
    ElapsedTime robotTimer;

    Gamepad previous1;
    Gamepad previous2;

    double targetAngle = 0;

    @Override
    public void init() {
        kraken = new HardwareControl(hardwareMap);
        PIDTimer = new ElapsedTime();
        robotTimer = new ElapsedTime();
        previous1 = new Gamepad();
        previous2 = new Gamepad();
        robotState = RobotState.READY_POSITION;
        sideHookState = SideHookState.DOWN;
        Servo light = hardwareMap.get(Servo.class, "light");
        light.setPosition(0);
    }

    @Override
    public void start() {
        kraken.ptoRelease();
        kraken.outtakeVertical();
        kraken.intakeHover();
        kraken.intakeLinkageIn();
        kraken.openIntakeClaw();
        kraken.openOuttakeClaw();
        kraken.rotateIntake(0);
    }

    @Override
    public void loop() {

        kraken.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        switch (sideHookState) {
            case DOWN:
                if (gamepad2.left_bumper) {
                    kraken.setHangPower(1);
                    sideHookState = SideHookState.LIFTING;
                }
                break;
            case LIFTING:
                if (kraken.getHangPosition() > 5100) {
                    kraken.setHangPower(0);
                    sideHookState = SideHookState.UP;
                }
                break;
            case UP:
                break;
            case NEGATIVE_POWER:
                if (kraken.getHangPosition() < 100) {
                    kraken.setHangPower(0);
                    sideHookState = SideHookState.DONE;
                }
                break;
        }

        switch (robotState) {

            case READY_POSITION:
                if (gamepad1.cross && !previous1.cross) {
                    kraken.outtakeTransfer();
                    targetAngle = 0;
                    kraken.intakeLinkageOut();
                    kraken.intakeHover();
                    kraken.openIntakeClaw();
                    robotState = RobotState.EXTENDING_INTAKE;
                    robotTimer.reset();
                    kraken.setLiftPower(0);
                    kraken.resetLiftEncoder();
                    kraken.rotateIntake(0);
                }
                if (gamepad1.square && !previous1.square) {
                    kraken.intakeLinkageIn();
                    kraken.intakeBack();
                    kraken.openOuttakeClaw();
                    kraken.outtakeVertical();
                    robotState = RobotState.SPECIMEN_INTAKE;
                    robotTimer.reset();
                }
                if (gamepad2.right_bumper) {
                    kraken.outtakeHang();
                    kraken.intakeLinkageIn();
                    robotState = RobotState.READY_HANG;
                }
                break;

            case READY_HANG:
                if (gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1 && sideHookState == SideHookState.UP) {
                    // Retract all hang stuff
                    kraken.setHangPower(-1);
                    robotState = RobotState.PULLING_SIDE_HOOKS;
                }
                break;

            case PULLING_SIDE_HOOKS:
                if (kraken.getHangPosition() < 3300) {
                    kraken.setHangPower(-0.2);
                    kraken.setLiftPower(1);
                    robotState = RobotState.LIFTING_MAIN_HOOKS;
                }
                break;

            case LIFTING_MAIN_HOOKS:
                if (kraken.getLiftPosition() > 1500) {
                    kraken.ptoLock();
//                    kraken.setLiftPower(0);
                    kraken.ptoMotor.setPower(-1);
                    kraken.setLiftPower(-1);
                    robotState = RobotState.PULLING_MAIN_HOOKS;
                    sideHookState = SideHookState.NEGATIVE_POWER;
                }
                break;

            case PULLING_MAIN_HOOKS:
                if (kraken.getLiftPosition() < 5) {
                    kraken.ptoMotor.setPower(-0.2);
                }
                break;

            case SPECIMEN_INTAKE:
                if (kraken.isIntakeIn()) {
                    kraken.specimenGrab();
                }

                if (gamepad1.cross && !previous1.cross) {
                    kraken.clampOuttake();
                    robotState = RobotState.SPECIMEN_OUT;
                    kraken.specimenOut();
                }
                break;

            case SPECIMEN_OUT:
                if (gamepad1.cross && !previous1.cross) {
                    kraken.specimenScore();
                    robotState = RobotState.SPECIMEN_SCORE;
                }
                break;

            case SPECIMEN_SCORE:
                if (gamepad1.cross && !previous1.cross) {
                    kraken.openOuttakeClaw();
                    kraken.specimenGrab();
                    robotState = RobotState.SPECIMEN_INTAKE;
                }
                if (gamepad1.square) {
                    kraken.openOuttakeClaw();
                    kraken.outtakeVertical();
                    kraken.intakeHover();
                    robotState = RobotState.READY_POSITION;
                }
                break;

            case EXTENDING_INTAKE:
                if (robotTimer.seconds() > 0.4) {
                    robotState = RobotState.INTAKE_OUT;
                    kraken.intakeLinkageOut();
                }
                if (gamepad1.right_bumper && !previous1.right_bumper) {
                    targetAngle += 45;
                }
                if (gamepad1.left_bumper && !previous1.left_bumper) {
                    targetAngle -= 45;
                }
                kraken.rotateIntake(targetAngle);
                break;

            case INTAKE_OUT:
                if (gamepad1.left_trigger > 0.5 && previous1.left_trigger < 0.5) {
                    kraken.intakeLinkageIn();
                }
                if (gamepad1.right_trigger > 0.5 && previous1.left_trigger < 0.5) {
                    kraken.intakeLinkageOut();
                }
                if (gamepad1.right_bumper && !previous1.right_bumper) {
                    targetAngle += 45;
                }
                if (gamepad1.left_bumper && !previous1.left_bumper) {
                    targetAngle -= 45;
                }
                targetAngle = Math.max(targetAngle, -90);
                targetAngle = Math.min(targetAngle, 90);

                kraken.rotateIntake(targetAngle);

                if (gamepad1.cross) {
                    kraken.intakeDown();
                    robotTimer.reset();
                    robotState = RobotState.PECKING;
                }

                break;

            case PECKING:
                if (robotTimer.seconds() > 0.1) {
                    kraken.closeIntakeClaw();
                }
                if (robotTimer.seconds() > 0.3) {
                    kraken.sampleGrabbedPosition();
                    kraken.rotateIntake(0);
                }
                if (robotTimer.seconds() > 0.1 && gamepad1.cross && !previous1.cross) {
                    kraken.outtakeTransfer();
                    kraken.intakeTransfer();
                    kraken.intakeLinkageIn();
                    robotState = RobotState.TRANSFERRING;
                    intakeIn = false;
                    high = true;
                    robotTimer.reset();
                }
                if (robotTimer.seconds() > 0.1 && gamepad1.triangle && !previous1.triangle) {
                    kraken.outtakeTransfer();
                    kraken.intakeTransfer();
                    kraken.intakeLinkageIn();
                    robotState = RobotState.TRANSFERRING;
                    intakeIn = false;
                    high = false;
                    robotTimer.reset();
                }
                if (robotTimer.seconds() > 0.1 && gamepad1.circle) {
                    kraken.openIntakeClaw();
                    kraken.intakeHover();
                    kraken.intakeLinkageOut();
                    robotState = RobotState.INTAKE_OUT;
                }

                if (robotTimer.seconds() > 0.5) {
                    robotState = RobotState.SAMPLE_GRABBED;
                }

                break;

            case SAMPLE_GRABBED:
                if (robotTimer.seconds() > 0.1 && gamepad1.cross && !previous1.cross) {
                    kraken.outtakeTransfer();
                    kraken.intakeLinkageIn();
                    kraken.intakeTransfer();
                    robotState = RobotState.TRANSFERRING;
                    robotTimer.reset();
                    intakeIn = false;
                    high = true;
                }
                if (robotTimer.seconds() > 0.1 && gamepad1.triangle && !previous1.triangle) {
                    kraken.outtakeTransfer();
                    kraken.intakeLinkageIn();
                    kraken.intakeTransfer();
                    robotState = RobotState.TRANSFERRING;
                    robotTimer.reset();
                    intakeIn = false;
                    high = false;
                }

                if (robotTimer.seconds() > 0.1 && gamepad1.circle) {
                    kraken.openIntakeClaw();
                    kraken.intakeHover();
                    kraken.intakeLinkageOut();
                    robotState = RobotState.INTAKE_OUT;
                }
                break;

            case TRANSFERRING:
                if (gamepad1.square) {
                    kraken.intakeLinkageOut();
                    kraken.intakeHover();
                    kraken.openIntakeClaw();
                    robotState = RobotState.INTAKE_OUT;
                }
                if (kraken.isIntakeIn() && !intakeIn) {
                    kraken.closeOuttakeClaw();
                    intakeIn = true;
                    robotTimer.reset();
                }
                if (robotTimer.seconds() > 0.15 && intakeIn && high) {
                    kraken.openIntakeClaw();
                    kraken.outtakeVertical();
                    robotState = RobotState.LIFTING_HIGH;
                    motionProfileTimer.reset();
                }
                if (robotTimer.seconds() > 0.15 && intakeIn && !high) {
                    kraken.openIntakeClaw();
                    kraken.outtakeVertical();
                    robotState = RobotState.LIFTING_LOW;
                }
                break;

            case LIFTING_HIGH:

                // Sets motor power based off high bucket motion profile
                double positionError = kraken.getLiftPosTarget(motionProfileTimer.seconds()) - kraken.getLiftPosition();
                double velocityError = kraken.getLiftVelTarget(motionProfileTimer.seconds()) - kraken.getLiftVelocity();
                double stateFeedback = positionError * k1 + velocityError * k2;
                kraken.setLiftPower(stateFeedback);

                if (kraken.getLiftPosition() > 1100) {
                    kraken.outtakeScoreBucket();
                    kraken.setLiftPower(0.13); // Power to keep lift at top height
                    robotState = RobotState.READY_DROP_BUCKET;
                }
                break;

            case LIFTING_LOW:
                error = 300 - kraken.getLiftPosition();
                power = error * Kp + (error - lastError) / PIDTimer.seconds() * Kd + Kf;
                kraken.setLiftPower(power);
                if (kraken.getLiftPosition() > 250) {
                    kraken.outtakeScoreBucket();
                    kraken.setLiftPower(0.13);
                    robotState = RobotState.READY_DROP_LOW;
                }
                break;

            case READY_DROP_BUCKET:
                error = 1350 - kraken.getLiftPosition();
                power = error * Kp + (error - lastError) / PIDTimer.seconds() * Kd + Kf;
                kraken.setLiftPower(power);
                if (gamepad1.cross && !previous1.cross) {
                    kraken.openOuttakeClaw();
                    robotState = RobotState.DROPPING_BUCKET;
                    kraken.setLiftPower(0.013);
                    robotTimer.reset();
                }
                break;

            case READY_DROP_LOW:
                error = 300 - kraken.getLiftPosition();
                power = error * Kp + (error - lastError) / PIDTimer.seconds() * Kd + Kf;
                kraken.setLiftPower(power);
                if (gamepad1.cross && !previous1.cross) {
                    kraken.openOuttakeClaw();
                    robotState = RobotState.DROPPING_BUCKET;
                    kraken.setHangPower(0.013);
                    robotTimer.reset();
                }
                break;

            case DROPPING_BUCKET:
                if (robotTimer.seconds() > 0.3) {
                    kraken.outtakeTransfer();
                    robotState = RobotState.RETRACTING_LIFT;
                    kraken.intakeLinkageOut();
                    robotTimer.reset();
                    kraken.setLiftPower(-1);
                }
                break;

            case RETRACTING_LIFT:
                kraken.intakeLinkageOut();
                if (kraken.getLiftPosition() < 20) {
                    kraken.setLiftPower(0);
                    kraken.intakeLinkageOut();
                    robotState = RobotState.READY_POSITION;
                }
                break;

        }

        previous1.copy(gamepad1);
        previous2.copy(gamepad2);

    }
}