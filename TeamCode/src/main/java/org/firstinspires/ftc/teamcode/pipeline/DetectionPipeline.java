package org.firstinspires.ftc.teamcode.pipeline;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class DetectionPipeline extends OpenCvPipeline {


    // Solo tenemos que buscar el pato amarillo e.g. como buscar los rings

    Mat Cb = new Mat();
    Mat YCrCb = new Mat();
    Mat tholdMat = new Mat();

    Scalar GRAY = new Scalar(220, 220, 220);
    Scalar GREEN = new Scalar(0, 255, 0);

    public int position = 0; // 0 izquierda, 1 centro, 2 derecha

    public static int squareLengthPX = 40;

    public static int globalY = 130;
    public static int startingX = 60;
    public static int gap = 60;


    @Override
    public Mat processFrame(Mat input) {

        Point firstSquareCorner = new Point(startingX, globalY);
        Point secondSquareCorner = new Point(startingX + squareLengthPX + gap, globalY);
        Point thirdSquareCorner = new Point(startingX + squareLengthPX * 2 + 2 * gap, globalY);

        // Procesar imagen
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_BGR2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
        Imgproc.threshold(Cb, tholdMat, 150, 255, Imgproc.THRESH_BINARY);

        int leftTotal = 0, middleTotal = 0, rightTotal = 0;


        // Ver cuantos pixeles blancos hay en la izquierda
        for (int y = (int) firstSquareCorner.y; y < (int) firstSquareCorner.y + squareLengthPX; y++) {
            for (int x = (int) firstSquareCorner.x; x < (int) firstSquareCorner.x + squareLengthPX; x++) {
                leftTotal += tholdMat.get(y,x)[0];
            }
        }

        // Ver cuantos pixeles blancos hay en la izquierda
        for (int y = (int) secondSquareCorner.y; y < (int) secondSquareCorner.y + squareLengthPX; y++) {
            for (int x = (int) secondSquareCorner.x; x < (int) secondSquareCorner.x + squareLengthPX; x++) {
                middleTotal += tholdMat.get(y,x)[0];
            }
        }

        // Ver cuantos pixeles blancos hay en la izquierda
        for (int y = (int) thirdSquareCorner.y; y < (int) thirdSquareCorner.y + squareLengthPX; y++) {
            for (int x = (int) thirdSquareCorner.x; x < (int) thirdSquareCorner.x + squareLengthPX; x++) {
                rightTotal += tholdMat.get(y,x)[0];
            }
        }

        // Dibujar en pantalla
        Imgproc.rectangle(
                input,
                firstSquareCorner,
                new Point(firstSquareCorner.x + squareLengthPX, firstSquareCorner.y + squareLengthPX),
                GRAY,
                1
        );

        Imgproc.rectangle(
                input,
                secondSquareCorner,
                new Point(secondSquareCorner.x + squareLengthPX, secondSquareCorner.y + squareLengthPX),
                GRAY,
                1
        );

        Imgproc.rectangle(
                input,
                thirdSquareCorner,
                new Point(thirdSquareCorner.x + squareLengthPX, thirdSquareCorner.y + squareLengthPX),
                GRAY,
                1
        );

        if (leftTotal > middleTotal && leftTotal > rightTotal) {
            position = 0;
            Imgproc.rectangle(
                    input,
                    firstSquareCorner,
                    new Point(firstSquareCorner.x + squareLengthPX, firstSquareCorner.y + squareLengthPX),
                    GREEN,
                    1
            );
        }
        else if (middleTotal > leftTotal && middleTotal > rightTotal) {
            position = 1;
            Imgproc.rectangle(
                    input,
                    secondSquareCorner,
                    new Point(secondSquareCorner.x + squareLengthPX, secondSquareCorner.y + squareLengthPX),
                    GREEN,
                    1
            );
        }
        else if (rightTotal > middleTotal && rightTotal > leftTotal) {
            position = 2;
            Imgproc.rectangle(
                    input,
                    thirdSquareCorner,
                    new Point(thirdSquareCorner.x + squareLengthPX, thirdSquareCorner.y + squareLengthPX),
                    GREEN,
                    1
            );
        }

        return input;
    }

}