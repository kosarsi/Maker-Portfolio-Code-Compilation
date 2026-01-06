package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="RUN")
public class RUN extends LinearOpMode {

    HardwareControl kraken;

    Orientation angles;
    double x, y, turn;
    double offset, heading, powGoal;

    boolean hooksUp = false;
    boolean intakeStalled = false;
    ElapsedTime stallTimer = new ElapsedTime();
    boolean intakePowered = false;
    boolean intakeWasOn = false;
    int stallCount = 0;

    public enum IntakeState {
        INTAKE_READY,
        INTAKE_RISING,
        INTAKE_RETRACTING,
        READY_TRANSFER,
        GATE_OPENING,
        TRANSFERRING,
        READY_EXTEND,
        INTAKE_EXTENDING,
        INTAKE_LOWERING,
        RESET_GOING_UP,
        RESET_RETRACTING
    }

    public enum LiftState {
        READY_TRANSFER,
        LIFTING_BEFORE_PIVOT,
        PIVOTING_OUT,
        READY_SCORE,
        PIVOTING_IN,
        GOING_BACK_DOWN,
        RESET_PIVOTING_IN,
        RESET_RETRACTING
    }

    public enum DroneState {
        READY_FIRE,
        ANGLING_UP,
        FIRING,
        GOING_DOWN
    }

    AnalogInput ultrasonic0;


    @Override
    public void runOpMode() throws InterruptedException {
        kraken = new HardwareControl();
        kraken.initTele(hardwareMap);
        kraken.intakeLiftUp();
        ultrasonic0 = hardwareMap.get(AnalogInput.class, "ultrasonic0");

        final int INTAKE_DOWN_POINT = -200;
        final int EXTENSION_LIMIT = -1150;
        final int LIFT_PIVOT_POINT = -200;
        final int LIFT_LIMIT = -750;
        final int INTAKE_DOWN_MORE = -300;

        Gamepad current1 = new Gamepad();
        Gamepad current2 = new Gamepad();
        Gamepad previous1 = new Gamepad();
        Gamepad previous2 = new Gamepad();

        int extensionTargetPosition = 0;
        int liftTargetPosition = 0;

        IntakeState intakeState = IntakeState.READY_EXTEND;
        LiftState liftState = LiftState.READY_TRANSFER;
        ElapsedTime intakeStateTimer = new ElapsedTime();
        ElapsedTime liftStateTimer = new ElapsedTime();
        ElapsedTime liftDeltaTime = new ElapsedTime();
        ElapsedTime extensionDeltaTime = new ElapsedTime();

        DroneState droneState = DroneState.READY_FIRE;
        ElapsedTime droneStateTimer = new ElapsedTime();

        boolean goingUp = false;

        boolean aReleased = false;
        boolean bReleased = false;

        boolean atBackdrop = false;

        int intakeHeight = 1;

        waitForStart();

        liftDeltaTime.reset();
        extensionDeltaTime.reset();

        while (opModeIsActive()) {
            previous1.copy(current1);
            previous2.copy(current2);
            current1.copy(gamepad1);
            current2.copy(gamepad2);

            // Drive Code
            angles = kraken.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            if (!atBackdrop) {
                x = Math.pow(gamepad1.left_stick_x, 3);
                y = Math.pow(gamepad1.left_stick_y, 3);
                turn = -Math.pow(gamepad1.right_stick_x, 3);
            } else {
                x = gamepad1.left_stick_x / 3;
                y = gamepad1.left_stick_y / 2;
                turn = -gamepad1.right_stick_x / 2;
                gamepad1.rumble(500);
            }
            if (intakeState == IntakeState.INTAKE_READY && kraken.extension.getCurrentPosition() < -750) {
                if(gamepad1.right_stick_button){
                    turn /= 1.5;
                } else {
                    turn /= 2.75;
                }
            }
            //Activate Cubic Drive
            //Activate Cubic Drive
            if (gamepad1.right_stick_button) {
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
            else kraken.drive(powGoal, heading + Math.PI, -1 * turn);

            //reset gyro
            if (gamepad1.y) {
                offset = Math.toRadians(180)+kraken.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            }

            // Lift code
            switch (liftState) {

                case READY_TRANSFER:
                    if (gamepad2.cross && intakeState != IntakeState.TRANSFERRING) {
                        kraken.lift.setPower(0.75);
                        liftTargetPosition = LIFT_PIVOT_POINT;
                        kraken.lift.setTargetPosition(liftTargetPosition);
                        kraken.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftState = LiftState.LIFTING_BEFORE_PIVOT;
                        kraken.clampA();
                        kraken.clampB();
                    }
                    break;

                case LIFTING_BEFORE_PIVOT:
                    if (kraken.lift.getCurrentPosition() <= LIFT_PIVOT_POINT + 10) {
                        kraken.basketOutMucho();
                        liftStateTimer.reset();
                        liftState = LiftState.PIVOTING_OUT;
                    }
                    break;

                case PIVOTING_OUT:
                    if (liftStateTimer.seconds() > 0.75) {
                        liftState = LiftState.READY_SCORE;
                        liftDeltaTime.reset();
                    }
                    break;

                case READY_SCORE:
                    atBackdrop = ultrasonic0.getVoltage() * 80 - 9.8 <= 10);

                    liftTargetPosition += gamepad2.left_stick_y * 700 * liftDeltaTime.seconds();
                    liftDeltaTime.reset();
                    if (liftTargetPosition < LIFT_LIMIT) {
                        liftTargetPosition = LIFT_LIMIT;
                    }
                    if (liftTargetPosition > LIFT_PIVOT_POINT) {
                        liftTargetPosition = LIFT_PIVOT_POINT;
                    }

                    kraken.lift.setTargetPosition(liftTargetPosition);
                    kraken.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if (kraken.lift.getCurrentPosition() < -300) {
                        kraken.basketOutUpMore();
                    } else {
                        kraken.basketOutMucho();
                    }

                    if (current2.circle && !previous2.circle) {
                        kraken.rotateBasket();
                    }

                    if (gamepad2.square) {
                        kraken.releaseA();
                        kraken.releaseB();
                        aReleased = true;
                        bReleased = true;
                    }

                    if (gamepad2.left_bumper) {
                        kraken.angleBasketLeft();
                        kraken.rotateTarget = 0.468;
                    }

                    if (gamepad2.right_bumper) {
                        kraken.angleBasketRight();
                        kraken.rotateTarget = 0.912;
                    }
                    if (current2.triangle && !previous2.triangle) {
                        if (kraken.rotateTarget == 0.025) {
                            if (aReleased) {
                                kraken.releaseB();
                                bReleased = true;
                            } else {
                                kraken.releaseA();
                                aReleased = true;
                            }
                        } else {
                            if (bReleased) {
                                kraken.releaseA();
                                aReleased = true;
                            } else {
                                kraken.releaseB();
                                bReleased = true;
                            }
                        }
                    }

                    if (aReleased && bReleased && gamepad2.cross) {
                        aReleased = false;
                        bReleased = false;
                        if (kraken.rotateTarget == 0.468 || kraken.rotateTarget == 0.912) {
                            kraken.rotateBasket();
                        }
                        kraken.releaseA();
                        kraken.releaseB();
                        liftState = LiftState.PIVOTING_IN;
                        kraken.basketIn();
                        liftStateTimer.reset();
                    }
                    break;

                case PIVOTING_IN:
                    atBackdrop = false;
                    kraken.releaseA();
                    kraken.releaseB();
                    if (liftStateTimer.seconds() > 0.5) {
                        kraken.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        kraken.lift.setPower(0.6);
                        liftState = LiftState.GOING_BACK_DOWN;
                    }
                    break;

                case GOING_BACK_DOWN:
                    if (kraken.liftDown()) {
                        liftState = LiftState.READY_TRANSFER;
                        liftTargetPosition = 0;
                        kraken.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        kraken.lift.setPower(0);
                    }
                    break;

                case RESET_PIVOTING_IN:
                    if (liftStateTimer.seconds() > 0.5) {
                        kraken.lift.setPower(0.5);
                        liftState = LiftState.RESET_RETRACTING;
                    }
                    break;

                case RESET_RETRACTING:
                    if (kraken.liftDown()) {
                        kraken.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        kraken.lift.setPower(0);
                        liftState = LiftState.READY_TRANSFER;
                    }
            }

            // Extension code
            switch (intakeState) {

                case INTAKE_READY:

                    if (gamepad1.right_bumper) {
                        intakePowered = true;
                        intakeStalled = false;
                    }
                    if (gamepad1.left_bumper) {
                        intakePowered = false;
                    }
                    if (gamepad1.left_trigger > 0.5) {
                        kraken.intakeReverse();
                    } else {
                        if(!intakeStalled) {
                            if (intakePowered) {
                                kraken.intakeOn();
                            } else {
                                kraken.intakeOff();
                            }
                        }
                    }

                    if (kraken.intake.getCurrent(CurrentUnit.MILLIAMPS) > 8000) {
                        kraken.intakeOff();
                        stallTimer.reset();
                        intakeStalled = true;
                    }

                    if(intakeStalled){ // && stallCount < 3
                        if(stallTimer.time()<0.1 ) {
                            kraken.intakeSoftReverse();
                        } else{
                            intakeStalled = false;
                            stallCount++;
                        }

                    }

                    telemetry.addData("state", intakeStalled);
                    telemetry.addData("current", kraken.intake.getCurrent(CurrentUnit.MILLIAMPS));
                    telemetry.addData("time", stallTimer.time());

                    telemetry.addData("state", intakeStalled);
                    telemetry.addData("current", kraken.intake.getCurrent(CurrentUnit.MILLIAMPS));
                    telemetry.addData("time", stallTimer.time());


                    if (current1.circle && !previous1.circle) {
                        intakeHeight -= 1;
                    }

                    if (current1.square && ! previous1.square) {
                        intakeHeight += 1;
                    }

                    if (gamepad1.cross) {
                        intakeHeight = 1;
                    }

                    if (intakeHeight < 1) {
                        intakeHeight = 1;
                    } else if (intakeHeight > 5) {
                        intakeHeight = 5;
                    }

                    if (intakeHeight == 1) {
                        kraken.intakeLiftDown();
                    } else if (intakeHeight == 2) {
                        kraken.intakeHeight2();
                    } else if (intakeHeight == 3) {
                        kraken.intakeHeight3();
                    } else if (intakeHeight == 4) {
                        kraken.intakeHeight4();
                    } else {
                        kraken.intakeHeight5();
                    }

                    kraken.extension.setPower(0.75);
                    if (gamepad2.right_trigger > 0) {
                        extensionTargetPosition -= gamepad2.right_trigger * 1080 * extensionDeltaTime.seconds();

                    }
                    if (gamepad2.left_trigger > 0) {
                        extensionTargetPosition += gamepad2.left_trigger * 1080 * extensionDeltaTime.seconds();
                    }
                    extensionDeltaTime.reset();

                    if (extensionTargetPosition > INTAKE_DOWN_POINT) {
                        extensionTargetPosition = INTAKE_DOWN_POINT;
                    }
                    if (extensionTargetPosition < EXTENSION_LIMIT) {
                        extensionTargetPosition = EXTENSION_LIMIT;
                    }

                    if (gamepad1.right_trigger > 0.5) {
                        extensionTargetPosition = INTAKE_DOWN_MORE;
                        kraken.intake.setPower(-0.8);
                        kraken.intakeLiftUp();
                        intakeState = IntakeState.INTAKE_RISING;
                    }

                    kraken.extension.setTargetPosition(extensionTargetPosition);
                    kraken.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    break;

                case INTAKE_RISING:
                    if (kraken.isIntakeUp()) {
                        extensionTargetPosition = 80;
                        kraken.extension.setPower(1);
                        kraken.extension.setTargetPosition(extensionTargetPosition);
                        kraken.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intakeState = IntakeState.INTAKE_RETRACTING;
                    }

                    break;

                case INTAKE_RETRACTING:
                    if (kraken.extensionIn()) {
                        extensionTargetPosition = 0;
                        kraken.extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        kraken.extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        kraken.extension.setPower(0.3);
                        intakeState = IntakeState.READY_TRANSFER;
                    }

                    break;

                case READY_TRANSFER:
                    if (liftState == LiftState.READY_TRANSFER) {
                        kraken.intakeOff();
                        kraken.openIntakeGate();
                        intakeState = IntakeState.GATE_OPENING;
                        intakeStateTimer.reset();
                    }

                    break;

                case GATE_OPENING:
                    if (intakeStateTimer.seconds() > 0.5) {
                        kraken.intakeTransfer();
                        kraken.releaseA();
                        kraken.releaseB();
                        intakeState = IntakeState.TRANSFERRING;
                    }
                    break;

                case TRANSFERRING:
                    if (kraken.rotateTarget == 0.69) {
                        if (kraken.beamABroken()) {
                            kraken.clampA();
                        }
                        if (kraken.beamABroken() && kraken.beamBBroken()) {
                            kraken.clampB();
                            intakeState = IntakeState.READY_EXTEND;
                        }
                    } else {
                        if (kraken.beamBBroken()) {
                            kraken.clampB();
                        }
                        if (kraken.beamABroken() && kraken.beamBBroken()) {
                            kraken.clampA();
                            intakeState = IntakeState.READY_EXTEND;
                        }
                    }

                    if (gamepad1.cross || gamepad1.right_trigger > 0.5) {
                        intakeState = IntakeState.READY_EXTEND;
                    }
                    break;

                case READY_EXTEND:
                    kraken.intakeOff();
                    extensionTargetPosition = 0;
                    kraken.extension.setTargetPosition(extensionTargetPosition);
                    kraken.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    kraken.extension.setPower(0.3);
                    if (current1.right_trigger > 0.5 && previous1.right_trigger < 0.5) {
                        kraken.extension.setPower(0.75);
                        extensionTargetPosition = INTAKE_DOWN_POINT - 10;
                        kraken.extension.setTargetPosition(extensionTargetPosition);
                        kraken.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        intakeState = IntakeState.INTAKE_EXTENDING;
                    }
                    break;

                case INTAKE_EXTENDING:
                    if (kraken.extension.getCurrentPosition() < INTAKE_DOWN_POINT) {
                        kraken.intakeLiftDown();
                        intakeStateTimer.reset();
                        intakeState = IntakeState.INTAKE_LOWERING;
                    }
                    break;

                case INTAKE_LOWERING:
                    if (intakeStateTimer.seconds() > 0.5) {
                        kraken.closeIntakeGate();
                        intakeState = IntakeState.INTAKE_READY;
                        extensionDeltaTime.reset();
                        kraken.intakeOn();
                        intakePowered = true;
                        intakeHeight = 1;
                    }
                    break;

                case RESET_GOING_UP:
                    kraken.intakeLiftUp();
                    if (kraken.isIntakeUp()) {
                        kraken.extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        kraken.extension.setPower(0.75);
                        intakeState = IntakeState.RESET_RETRACTING;
                    }
                    break;

                case RESET_RETRACTING:
                    if (kraken.extensionIn()) {
                        kraken.extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        kraken.extension.setPower(0);
                        intakeState = IntakeState.READY_EXTEND;
                    }
            }

            // Extension reset
            if (current1.triangle && !previous1.triangle) {
                intakeState = IntakeState.RESET_GOING_UP;
            }

            // Drone
            switch (droneState) {
                case READY_FIRE:
                    if (gamepad1.ps && gamepad2.ps) {
                        kraken.hookDrone();
                        droneStateTimer.reset();
                        droneState = DroneState.ANGLING_UP;
                    }
                    break;
                case ANGLING_UP:
                    if (droneStateTimer.seconds() > 0.25) {
                        kraken.releaseDrone();
                        droneState = DroneState.FIRING;
                        droneStateTimer.reset();
                    }
                    break;
                case FIRING:
                    if (droneStateTimer.seconds() > 0.4) {
                        kraken.hooksDownAfterDrone();
                        droneState = DroneState.GOING_DOWN;
                        droneStateTimer.reset();
                    }
                    break;
                case GOING_DOWN:
                    if (droneStateTimer.seconds() > 0.5) {
                        droneState = DroneState.READY_FIRE;
                    }
                    break;
            }

            // Hook
            if (gamepad2.dpad_up && gamepad2.ps) {
                kraken.hooksUp();
                hooksUp = true;
                telemetry.addData("Hooks", "Up");
            }
            if (gamepad2.dpad_right && gamepad2.ps) {
                kraken.hooksDown();
                telemetry.addData("Hooks", "Down");
            }

            if (gamepad2.dpad_down && gamepad2.ps && hooksUp) {
                kraken.hook.setPower(1);
                goingUp = true;
            } else {
                if (goingUp) {
                    kraken.hook.setPower(0.2);
                } else {kraken.hook.setPower(0);}
            }

            if (gamepad2.options && !previous2.options) {
                liftState = LiftState.RESET_PIVOTING_IN;
                kraken.basketIn();
                liftStateTimer.reset();
            }

            telemetry.addData("Lift state", liftState);
            telemetry.addData("Extension state", intakeState);
            telemetry.addData("Beam A Broken", kraken.beamA.getState());
            telemetry.addData("Beam B Broken", kraken.beamB.getState());
            telemetry.addData("Ultrasonic 0", ultrasonic0.getVoltage());


            telemetry.update();

        }
    }
}