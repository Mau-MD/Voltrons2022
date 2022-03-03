package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Driver", group = "opmode")
public class Teleop extends LinearOpMode {

    // Primero declaramos todas las variables que vamos a usar
    // ( Motores, servos y temporizadores)

    DcMotor leftFront;
    DcMotor rightFront;

    DcMotor leftBack;
    DcMotor rightBack;

    DcMotor duckArm;
    DcMotor arm;

    Servo leftClaw;
    Servo rightClaw;
    Servo ankle;

    public static double leftClawOpen = 0.35;
    public static double leftClawClosed = 0;

    public static double rightClawOpen = 0.65;
    public static double rightClawClosed = 1;

    public static double ankleDivider = 100;

    boolean open = false;

    ElapsedTime aButton = new ElapsedTime();
    ElapsedTime bButton = new ElapsedTime();
    ElapsedTime xButton = new ElapsedTime();
    ElapsedTime yButton = new ElapsedTime();

    ElapsedTime armDelay = new ElapsedTime();




    @Override
    public void runOpMode() throws InterruptedException {

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

        // Reiniciamos los temporizadores
        aButton.reset();
        xButton.reset();
        yButton.reset();
        bButton.reset();
        armDelay.reset();

        double invert = 1;
        double adjust = 10;
        double k_p = 0.003;

        double position_goal = arm.getCurrentPosition();
        double anklePosition = 0.39;

        waitForStart();
        while (opModeIsActive()) {

            // Invert puede tener dos valores: 1 o -1. Al mutiplicar invert por eso, invertimos el poder que se le debe de asignar al motor

            // Adjust es un numero que se dividira entre 10, para generar un numero decimal (e.g. 0.5) entonces al multiplicar todo el valor por este, se reducira a la mitad el poder de las llantas

            rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.right_stick_x * -invert)) * (adjust / 10.0));
            leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * -invert)) * (adjust / 10.0));
            rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x * -invert)) * (adjust / 10.0));
            leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * -invert)) * (adjust / 10.0));

            // Slow Mode
            if (gamepad1.a && aButton.milliseconds() > 300) {
                if (adjust == 10) {
                    adjust = 4;
                } else {
                    adjust = 10;
                }
                aButton.reset();
            }

            // Invert Mode
            if (gamepad1.y && yButton.milliseconds() > 300) {
                if (invert == 1) {
                    leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
                    rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
                    leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
                    rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
                    invert = -1;
                } else {
                    leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                    rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
                    leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
                    rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
                    invert = 1;
                }
                yButton.reset();
            }

            // Los bumpers de un control, regresan un valor entre 0 y 1. Entonces tenemos que asignar poder dependiendo de eso
            if (gamepad1.right_bumper) {
                duckArm.setPower(0.05);
            } else if (gamepad1.left_bumper) {
                duckArm.setPower(-0.05);
            } else {
                // Si ninguno de los dos botones estan siendp presionados, lo apagamos
                duckArm.setPower(0);
            }


            // Jugador 2

            // Esto de los miliseconds, es un truco porque como el codigo se actauliza muy rapido, el darle un click a la x, haria que se corriera este pedazo de codigo como unas 15 veces. Haciendo un delay de 300 milisegundos evita esto
            if (gamepad2.x && xButton.milliseconds() > 300) {
                if (open) {
                    leftClaw.setPosition(leftClawClosed);
                    rightClaw.setPosition(rightClawClosed);
                    open = false;
                } else {
                    leftClaw.setPosition(leftClawOpen);
                    rightClaw.setPosition(rightClawOpen);
                    open = true;
                }
                xButton.reset();
            }


            // Brazo
            if (gamepad2.left_trigger > 0 && armDelay.milliseconds() > 20) {
                position_goal += 8;
                armDelay.reset();
            }
            if (gamepad2.right_trigger > 0 && armDelay.milliseconds() > 20) {
                position_goal -= 8;
                armDelay.reset();
            }

            // Muneca
            anklePosition += -gamepad2.left_stick_y / ankleDivider;
            anklePosition = Range.clip(anklePosition, 0, 1.0);
            ankle.setPosition(anklePosition);

            double error = position_goal - arm.getCurrentPosition();
            double armPower = k_p * error;

            arm.setPower(-armPower);

            telemetry.addData("Duck Power", duckArm.getPower());
            telemetry.addData("Invert", invert);
            telemetry.addData("Left Claw Position", leftClaw.getPosition());
            telemetry.addData("Right Claw Position", rightClaw.getPosition());
            telemetry.addData("Ankle", ankle.getPosition());
            telemetry.addData("Error", error);
            telemetry.addData("Arm Power", armPower);
            telemetry.addData("Arm position", arm.getCurrentPosition());
            telemetry.addData("Position Goal", position_goal);
            telemetry.addData("Slow Mode", adjust == 4);
            telemetry.update();

        }
    }
}
