package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pipeline.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@TeleOp(name="Blue Wobble", group="auto")
public class BlueWobble extends LinearOpMode {

    DcMotor leftFront;
    DcMotor rightFront;

    DcMotor leftBack;
    DcMotor rightBack;

    DcMotor duckArm;
    DcMotor arm;

    Servo leftClaw;
    Servo rightClaw;
    Servo ankle;

    public static double gyroKp = 0.003;
    public static double armKp = 0.003;

    public static double strafeTime = 2000;


    DetectionPipeline visionPipeline;

    public static double leftClawOpen = 0.35;
    public static double leftClawClosed = 0;

    public static double rightClawOpen = 0.65;
    public static double rightClawClosed = 1;

    public static double ankleDown = 0.4;
    public static double ankleUp = 0.6;

    @Override
    public void runOpMode() {

        // Luego las asignamos a su respectivo pedazo de hardware
        leftFront = hardwareMap.dcMotor.get("fl");
        rightFront = hardwareMap.dcMotor.get("fr");
        leftBack = hardwareMap.dcMotor.get("bl");
        rightBack = hardwareMap.dcMotor.get("br");

        duckArm = hardwareMap.dcMotor.get("duck");
        arm = hardwareMap.dcMotor.get("arm");

        leftClaw = hardwareMap.servo.get("lcl");
        rightClaw = hardwareMap.servo.get("rcl");

        ankle = hardwareMap.servo.get("ankle");

        // Invertimos los motores de frabrica
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hacemos esto para que por defecto, cuando alguien deje de mover el stick de motor, se frenen todos los motores y no se quede patinando
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        // Creando errores
        visionPipeline = new DetectionPipeline();
        phoneCam.setPipeline(visionPipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Imu
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addLine("Opening Camera...");
        telemetry.update();



        double positionGoal = arm.getCurrentPosition();
        Drive drive = new Drive(leftFront, leftBack, rightFront, rightBack, imu);

        telemetry.addLine("Waiting for start.");
        telemetry.update();


        leftClaw.setPosition(leftClawOpen);
        rightClaw.setPosition(rightClawOpen);

        waitForStart();

        if (opModeIsActive()) {

            drive.leftTimed(0.3, 700, 180, gyroKp);

            drive.frontTimed(0.3, 900, 180, gyroKp);

            drive.setOrientation(0.4, 90, 3000);

            drive.frontTimed(0.3, 700, 90, gyroKp);

            ankle.setPosition(ankleDown);

            sleep(2000);

            leftClaw.setPosition(leftClawClosed);
            rightClaw.setPosition(rightClawClosed);

            sleep(2000);

            ankle.setPosition(ankleUp);

            drive.backTimed(0.3, 1150, 90, gyroKp);

            drive.setOrientation(0.4, 180, 3000);

            drive.backTimed(0.6, 2400, 180, gyroKp * 2);

        }
    }
}
