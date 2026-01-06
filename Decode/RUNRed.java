package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="RUN_RED")
public class RUNRed extends OpMode {

    public static double spinningP = 0.006;
    public static double spinningI = 0.00000245;
    public static double spinningD = 0.00003;
    public static double spinningF = 0.00055;

    ElapsedTime validTimer;

    public static double shootingP = 0.00845;
    public static double shootingI = 0.00000115;
    public static double shootingD = 0.000044;
    public static double shootingF = 0.00055;

    HardwareControl kraken;
    static double targetRPM;

    String robotState;
    ElapsedTime pidTimer;
    double lastError;
    double totalError;

    FtcDashboard dashboard;

    double shooterCurrent;

    @Override
    public void init() {
        kraken = new HardwareControl(hardwareMap, "red");
        robotState = "intake";
        pidTimer = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
        validTimer = new ElapsedTime();
    }

    @Override
    public void start() {
        kraken.setPinpointPose();
        kraken.setTurret(0.5);
        kraken.closeGate();
        kraken.setHoodPosition(0.0);
    }

    @Override
    public void loop() {

        double derivative = 0.0;
        double hoodAngle = kraken.getHoodAngle(kraken.getTargetDistance());
        kraken.setHoodPosition(hoodAngle);
        targetRPM = kraken.getTargetRPM(kraken.getTargetDistance());

        kraken.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        kraken.updatePose();
        targetRPM = kraken.getTargetRPM(kraken.getTargetDistance());

        // Reset robot dead wheel position if tag not initially visible
        if (gamepad1.ps) {
            kraken.setPinpointPose();
        }

        if (robotState.equals("intake")) {
            kraken.setShooterPower(0.0);
            if (gamepad1.right_bumper) {
                kraken.intakeOn();
                kraken.transferOn();
            }
            if (gamepad1.left_bumper) {
                kraken.intakeOff();
                kraken.transferOff();
            }
            if (gamepad1.left_trigger > 0.5) {
                kraken.intakeReverse();
                kraken.transferReverse();
            }
            // Controller haptic feedback to let driver know that intake has 3 balls
            if (kraken.intakeFull()) {
                gamepad1.rumble(500);
            }
            if (gamepad1.right_trigger > 0.5) {
                robotState = "spinning_up";
                kraken.intakeOff();
                kraken.transferOff();
                pidTimer.reset();
                lastError = 0.0;
                totalError = 0.0;
            }
        } else if (robotState.equals("spinning_up")) {
            double x = kraken.getPose().getX(DistanceUnit.INCH);
            double y = kraken.getPose().getY(DistanceUnit.INCH);
            double theta = kraken.getPose().getHeading(AngleUnit.DEGREES);
            if (theta > 180) {
                theta = 360 - theta;
            }
            double angle = Math.atan(y / Math.abs(x)) * 180 / Math.PI;
            kraken.setTurretAngle(-theta - angle);

            double currentRPM = kraken.getShooterVelocity();
            double dt = pidTimer.seconds();

            double error = targetRPM - currentRPM;
            totalError += error * dt;

            if (error < 300) {
                kraken.openGate();
            }

            derivative = (error - lastError) / dt;

            double power = spinningP * error +
                    spinningI * totalError +
                    spinningD * derivative +
                    spinningF * targetRPM * 12.0 / kraken.getRobotVoltage();

            kraken.setShooterPower(power);

            lastError = error;
            pidTimer.reset();
            shooterCurrent = kraken.getShooterCurrent();
            // Fire the balls out
            if (gamepad1.cross && Math.abs(error) < 50) {
                robotState = "shooting";
            }
            // Reset if button was pressed prematurely
            if (gamepad1.circle) {
                robotState = "intake";
                kraken.closeGate();
                kraken.setShooterPower(0.0);
                kraken.intakeOff();
                kraken.transferOff();
            }
        } if (robotState.equals("shooting")) {

            // Setting turret angle
            double x = kraken.getPose().getX(DistanceUnit.INCH);
            double y = kraken.getPose().getY(DistanceUnit.INCH);
            double theta = kraken.getPose().getHeading(AngleUnit.DEGREES);
            if (theta > 180) {
                theta = 360 - theta;
            }
            double angle = Math.atan(y / Math.abs(x)) * 180 / Math.PI;
            kraken.setTurretAngle(-theta - angle);

            shooterCurrent = 0.7 * shooterCurrent + kraken.getShooterCurrent() * 0.3;

            targetRPM = kraken.getTargetRPM(kraken.getTargetDistance());

            double currentRPM = kraken.getShooterVelocity();
            double dt = pidTimer.seconds();

            double error = targetRPM - currentRPM;
            totalError += error * dt;

            derivative = (error - lastError) / dt;

            double power = shootingP * error +
                    shootingI * totalError +
                    shootingD * derivative +
                    shootingF * targetRPM * 12.0 / kraken.getRobotVoltage();

            kraken.setShooterPower(power);

            lastError = error;
            pidTimer.reset();
            if (validTimer.seconds() > 0.065 && Math.abs(error) < 25) {
                kraken.intakeOn();
                kraken.transferOn();
            } else {
                kraken.intakeOff();
                kraken.transferOff();
            }
            if (gamepad1.circle) {
                robotState = "intake";
                kraken.setShooterPower(0.0);
                kraken.closeGate();
                kraken.intakeOff();
                kraken.transferOff();
            }
        }
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Current RPM", kraken.getShooterVelocity());
        packet.put("Target RPM", targetRPM);
        packet.put("Derivative", derivative);
        packet.put("Shooter Current", kraken.getShooterVelocity());
        dashboard.sendTelemetryPacket(packet);

    }
}