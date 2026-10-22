package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "teleop2", group = "TeleOp")
public class mecanumautocentering extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, armMotor;
    private DcMotor parallelEncoder, perpendicularEncoder;
    private IMU imu;
    private Limelight3A limelight;

    private double robotX = 0.0, robotY = 0.0, robotHeading = 0.0;
    private int lastParTicks = 0, lastPerpTicks = 0;

    private static final double TICKS_PER_INCH = 330.0;
    private static final double PARALLEL_OFFSET = -5.0;
    private static final double PERPENDICULAR_OFFSET = -4.0;

    private static final double ALIGN_KP = 0.025, MIN_ALIGN_POWER = 0.05, TARGET_THRESHOLD = 0.8;
    private static final double DISTANCE_KP = 0.035, MIN_DISTANCE_POWER = 0.05, DISTANCE_THRESHOLD = 0.5, TARGET_TY = 0.0;

    @Override
    public void runOpMode() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftdrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightdrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftdrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightdrive");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        parallelEncoder = frontLeftDrive;
        perpendicularEncoder = frontRightDrive;

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Statut", "Initialise et Corrige - Pret !");
        telemetry.update();

        waitForStart();

        lastParTicks = parallelEncoder.getCurrentPosition();
        lastPerpTicks = perpendicularEncoder.getCurrentPosition();

        while (opModeIsActive()) {
            updateOdometry2Wheels();

            if (gamepad1.a) {
                imu.resetYaw();
                robotX = 0.0; robotY = 0.0; robotHeading = 0.0;
                parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                perpendicularEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lastParTicks = 0; lastPerpTicks = 0;
            }

            double forward = -gamepad1.left_stick_y;
            double right   = gamepad1.left_stick_x;
            double rotate  = gamepad1.right_stick_x;

            LLResult result = limelight.getLatestResult();
            boolean autoAlignActive = false;
            long targetFiducialId = -1;

            if (gamepad1.x && result != null && result.isValid()) {
                if (!result.getFiducialResults().isEmpty()) {
                    targetFiducialId = result.getFiducialResults().get(0).getFiducialId();
                }

                if (targetFiducialId == 35 || targetFiducialId == 36) {
                    autoAlignActive = true;
                    double tx = result.getTx();
                    double ty = result.getTy();

                    if (Math.abs(tx) > TARGET_THRESHOLD) {
                        rotate = tx * ALIGN_KP;
                        if (Math.abs(rotate) < MIN_ALIGN_POWER) rotate = Math.signum(tx) * MIN_ALIGN_POWER;
                    } else {
                        rotate = 0;
                    }

                    double tyError = TARGET_TY - ty;
                    if (Math.abs(tyError) > DISTANCE_THRESHOLD) {
                        forward = tyError * DISTANCE_KP;
                        if (Math.abs(forward) < MIN_DISTANCE_POWER) forward = Math.signum(tyError) * MIN_DISTANCE_POWER;
                    } else {
                        forward = 0;
                    }
                    right = 0;
                }
            }

            if (autoAlignActive || gamepad1.left_bumper) {
                drive(forward, right, rotate);
            } else {
                driveFieldRelative(forward, right, rotate);
            }

            double armPower = gamepad1.right_trigger;
            armMotor.setPower(armPower);

            telemetry.addData("X (pouces)", "%.2f", robotX);
            telemetry.addData("Y (pouces)", "%.2f", robotY);
            telemetry.addData("Angle (deg)", "%.1f", Math.toDegrees(robotHeading));
            telemetry.update();
        }
        limelight.stop();
    }

    private void updateOdometry2Wheels() {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double deltaHeading = AngleUnit.normalizeRadians(currentHeading - robotHeading);

        int currentParTicks = parallelEncoder.getCurrentPosition();
        int currentPerpTicks = perpendicularEncoder.getCurrentPosition();

        double deltaParInches = (currentParTicks - lastParTicks) / TICKS_PER_INCH;
        double deltaPerpInches = (currentPerpTicks - lastPerpTicks) / TICKS_PER_INCH;

        double deltaRobotForward = deltaParInches - (deltaHeading * PARALLEL_OFFSET);
        double deltaRobotStrafe = deltaPerpInches - (deltaHeading * PERPENDICULAR_OFFSET);

        robotX += deltaRobotForward * Math.cos(currentHeading) - deltaRobotStrafe * Math.sin(currentHeading);
        robotY += deltaRobotForward * Math.sin(currentHeading) + deltaRobotStrafe * Math.cos(currentHeading);
        robotHeading = currentHeading;

        lastParTicks = currentParTicks;
        lastPerpTicks = currentPerpTicks;
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = AngleUnit.normalizeRadians(Math.atan2(forward, right) - robotHeading);
        double r = Math.hypot(right, forward);
        drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    public void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(br), Math.abs(bl)))));

        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backRightDrive.setPower(br / max);
        backLeftDrive.setPower(bl / max);
    }
}
