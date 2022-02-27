package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Imu {

    private BNO055IMU imu;


    /**
     * Initializes imu class given an imu
     *
     * @param imu imu
     */
    public Imu(BNO055IMU imu) {
        this.imu = imu;
    }

    /**
     * Gets the current heading of the robot. 180 -> 0 -> -180
     *
     * @return current heading
     */
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * Gets the current heading of the robot normalized to 0 -> 360 degrees
     *
     * @return current heading normalized to 360 degrees
     */
    public double getAngleNormalized() {
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle <= 0) {
            return 180 + Math.abs(angle);
        } else {
            return 180 - angle;
        }
    }

    /**
     * Gets the error between two angles
     *
     * @param angle current angle
     * @param goal  goal angle
     * @return error
     */
    public static double getError(double angle, double goal) {
        double error = goal - angle;
        while (error > 180) {
            error -= 360;
        }
        while (error < -180) {
            error += 360;
        }
        return error;
    }

    /**
     * Normalizes spline angle to the form 0 -> 360
     *
     * @param angle spline angle
     * @return normalized angle
     */
    public static double normalizeSplineAngle(double angle) {
        if (angle >= 0) {
            return 180 + (90 - angle);
        } else {
            return (270 + Math.abs(angle)) % 360;
        }
    }

}
