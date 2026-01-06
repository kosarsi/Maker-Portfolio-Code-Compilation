package org.firstinspires.ftc.teamcode.Tele;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class HardwareControl {

    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    public DcMotorEx intake;
    public DcMotorEx transfer;
    DcMotorEx shooter1;
    DcMotorEx shooter2;

    Servo hood;
    public Servo gate;
    Servo turret1;
    Servo turret2;

    DigitalChannel ball1;
    DigitalChannel ball2;
    DigitalChannel ball3;

    Limelight3A limelight3A;
    public GoBildaPinpointDriver pinpointDriver;

    VoltageSensor voltageSensor;

    public HardwareControl(HardwareMap hardwareMap, String allianceColor) {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");
        turret1 = hardwareMap.get(Servo.class, "turret1");

        ball1 = hardwareMap.get(DigitalChannel.class, "ball1");
        ball2 = hardwareMap.get(DigitalChannel.class, "ball2");
        ball3 = hardwareMap.get(DigitalChannel.class, "ball3");
        ball1.setMode(DigitalChannel.Mode.INPUT);
        ball2.setMode(DigitalChannel.Mode.INPUT);
        ball3.setMode(DigitalChannel.Mode.INPUT);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpointDriver.setOffsets(0, -3.89181102, DistanceUnit.INCH);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.resetPosAndIMU();

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        if (allianceColor.equals("blue")) {
            limelight3A.pipelineSwitch(1);
        } else {
            limelight3A.pipelineSwitch(2);
        }
        limelight3A.start();
    }

    public void setPinpointPose() {
        LLResult result = limelight3A.getLatestResult();
        if (result == null) {
            return;
        }
        if (result.getBotpose().getPosition().x == 0) {
            return;
        }
        Pose3D pose = result.getBotpose();
        double x = pose.getPosition().x * -39.37;
        double y = pose.getPosition().y * -39.37;
        double yaw = pose.getOrientation().getYaw();
        if (yaw < 0) {
            yaw += 180.0;
        } else {
            yaw -= 180.0;
        }
        Pose2D updatedPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, yaw);
        pinpointDriver.setPosition(updatedPose);
    }

    public double getPinpointVelocity() {
        return Math.sqrt(Math.pow(pinpointDriver.getVelX(DistanceUnit.INCH), 2) + Math.pow(pinpointDriver.getVelY(DistanceUnit.INCH), 2));
    }

    public Pose2D getPose() {
        return pinpointDriver.getPosition();
    }

    public void setTurretAngle(double angle) {
        setTurret(angle * -0.006 + 0.5);
    }

    public void updatePose() {
        LLResult result = limelight3A.getLatestResult();

        pinpointDriver.update();
        // If tag visible and reading is valid
        if (result != null && result.getBotpose().getPosition().x != 0) {
            Pose3D pose = result.getBotpose();
            // Get x and y relative to goal tag
            double x = pose.getPosition().x * -39.37;
            double y = pose.getPosition().y * -39.37;
            double yaw = pose.getOrientation().getYaw();
            // Limit yaw value to a useful range
            if (yaw < 0) {
                yaw += 180.0;
            } else {
                yaw -= 180.0;
            }
            Pose2D tagPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, yaw);
            Pose2D odoPose = pinpointDriver.getPosition();
            double velocity = getPinpointVelocity();
            // Only trust camera reading if velocity is less than a tuned threshold
            if (velocity < 30) {
                // Scale camera variance quadratically with velocity, low speed = low variance, 30 in/sec results in zero trust
                double cameraVariance = Math.pow(velocity, 2) / 900.0;
                double finalX = tagPose.getX(DistanceUnit.INCH) * (1-cameraVariance) + odoPose.getX(DistanceUnit.INCH) * cameraVariance;
                double finalY = tagPose.getY(DistanceUnit.INCH) * (1-cameraVariance) + odoPose.getY(DistanceUnit.INCH) * cameraVariance;
                // IMU heading doesn't require any correction from tag prior to initial localization
                double finalHeading = odoPose.getHeading(AngleUnit.DEGREES);
                Pose2D outputPose = new Pose2D(DistanceUnit.INCH, finalX, finalY, AngleUnit.DEGREES, finalHeading);
                pinpointDriver.setPosition(outputPose);
            }
        }
    }

    public void resetPose() {
        pinpointDriver.resetPosAndIMU();
    }

    public void setPose(Pose2D pose) {
        pinpointDriver.setPosition(pose);
    }

    public double getRobotVoltage() {
        return voltageSensor.getVoltage();
    }

    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    public boolean intakeFull() {
        return ball1.getState() && ball2.getState() && ball3.getState();
    }

    public double getShooterCurrent() {
        return shooter1.getCurrent(CurrentUnit.AMPS);
    }

    public void drive(double x, double y, double rx) {
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

    public void intakeOn() {
        intake.setPower(1);
    }

    public void intakeOff() {
        intake.setPower(0);
    }

    public void intakeReverse() {
        intake.setPower(-1);
    }

    public void transferIdle() {
        transfer.setPower(0.5);
    }

    public void transferOff() {
        transfer.setPower(0);
    }

    public void transferOn() {
        transfer.setPower(1);
    }

    public void transferReverse() {
        transfer.setPower(-1);
    }

    public void setTurret(double pos) {
        turret1.setPosition(pos);
    }

    public double getShooterVelocity() {
        return shooter2.getVelocity();
    }

    public void setShooterPower(double power) {
        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public void openGate() {
        gate.setPosition(0.3);
    }

    public void closeGate() {
        gate.setPosition(0.95);
    }

    public double getTargetRPM(double distance) {
        if (distance < 110) {
            return distance * distance * 0.1917 - distance * 21.25 + 1439;
        } else {
            return distance * distance * 0.09307 + distance * -17.3 + 1800;
        }
    }

    public double getHoodAngle(double distance) {
        if (distance < 110) {
            return 0.0;
        } else {
            return 0.5;
        }
    }

    public double getTargetDistance() {
        double x = pinpointDriver.getPosition().getX(DistanceUnit.INCH);
        double y = pinpointDriver.getPosition().getY(DistanceUnit.INCH);
        return Math.sqrt(x * x + y * y);
    }
}