package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drive {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    Imu imu;

    public Drive(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack, BNO055IMU imu) {
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
        this.imu = new Imu(imu);
    }

    public void idle() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public void frontTimed(double power, int ms) {

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < ms) {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }
        idle();
    }

    public void backTimed(double power, int ms) {

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < ms) {
            leftFront.setPower(-power);
            leftBack.setPower(-power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);
        }
        idle();
    }

    public void leftTimed(double power, int ms) {

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < ms) {
            leftFront.setPower(-power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(-power);
        }
        idle();
    }

    public void rightTimed(double power, int ms) {

        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.milliseconds() < ms) {
            leftFront.setPower(power);
            leftBack.setPower(-power);
            rightFront.setPower(-power);
            rightBack.setPower(power);
        }
    }

    public void setOrientation(double power, double angle, double kp) {

        double multiplier = 1;

        while (!Thread.currentThread().isInterrupted()) {

            double error = Imu.getError(imu.getAngleNormalized(), angle);
            double correction = error * kp;

            power *= multiplier;

            leftBack.setPower(power * correction);
            leftBack.setPower(power * correction);
            rightFront.setPower(-power * correction);
            rightBack.setPower(-power * correction);

            if (Math.abs(error) < 10) {
                // Hacer el robot que vaya mas lento
                multiplier = 0.4;
            }

            if (Math.abs(error) < 3) {
                // Romper cuando llegue a un angulo
                break;
            }
        }

        idle();
    }
}


