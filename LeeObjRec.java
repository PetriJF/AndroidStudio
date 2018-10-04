package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.ByteBuffer;


@Autonomous(name="DetectObject", group="Linear Opmode")
public class objectRecognition extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    int minD = 30000 / 4, objS, imageWidth, imageHeight, left = 0, center = 1, right = 2, scale = 4; //minD = minimal difference between 2 colors for comparision
    boolean[][] cube_color, lee_matrix;
    int[] Loc;
    @Override
    public void runOpMode() throws InterruptedException {

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "ASCPGNz/////AAABmduHqNxelkM0iYG1koMxB5ZrInt8LFjdO/quEXlhG4o5uE4U/EtrZaYczNr7G6TB88wl39Ajg2kXLOTpUnvDqfwba1sfK6+GSziuEvpDkI4ldc++m6P93BO2gbLjiS5unVSqzKvAHR8LAN11ZJh07YsIF8JAbDSjUtR0eRfJsHuU1sNyoP4yX+LKHRyvUAmGloYRsYItswmOD03kImxhfwREbXGP/eCEwPfO0Iv8kAZImUpdL+LjPcnEroUqVVMcUUjFU/ggn4Qt6sB7w2MOU7P1HooFtRXx7j6UK+S9Hlv0rUfMxdEoWbju6pTmaFril3igTHM1rIo5tWLu1EbeG3raK6myZG/pviMbYTjwfX0X";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        locale.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time


        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

                VuforiaLocalizer.CloseableFrame frame = locale.getFrameQueue().take(); //takes the frame at the head of the queue
                Image img = null;
                long numImages = frame.getNumImages();

                for (int i = 0; i < numImages; i++) {
                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888) {
                        img = frame.getImage(i);
                        break;
                    }
                }

                if (img != null) {

                    telemetry.addData("Status", "Start img processing: " + runtime.toString());
                    ByteBuffer pixels = img.getPixels();
                    byte[] pixelArray = new byte[pixels.remaining()];
                    pixels.get(pixelArray, 0, pixelArray.length);
                    imageWidth = img.getWidth();
                    imageHeight = img.getHeight();
                    int stride = img.getStride();
                    // telemetry.addData("Image", "Image width: " + imageWidth);
                    // telemetry.addData("Image", "Image height: " + imageHeight);
                    //telemetry.addData("Image", "Image stride: " + stride);
                    //telemetry.addData("Image", "First pixel byte: " + pixelArray[0]);
                    //telemetry.addData("Length", pixelArray.length);
                    // telemetry.addLine();
                    // telemetry.update();

                    /* finding where 0,0 and width, height is: */
                    int[] pixel = new int[5];
              /*  pixel = GetRGB(0, 0, pixelArray, imageWidth); ///dreapta sus
                ShowRGB(pixel);
                pixel = GetRGB(imageWidth - 1, imageHeight  - 1, pixelArray, imageWidth);///stanga jos
                ShowRGB(pixel);*/

                    cube_color = new boolean[imageWidth / scale][imageHeight / scale];
                    int scale2 = scale * scale;

                    for (int y = 0; y < imageHeight; y += scale)
                        for (int x = 0; x < imageWidth; x += scale)
                        {


                            pixel = GetRGB(x, y, pixelArray);

                            int sRed = 0;
                            int sGreen = 0;
                            int sBlue = 0;


                            for (int j = y; j < y + scale; ++j)
                                for (int i = x; i < x + scale; ++i)
                                {
                                    pixel = GetRGB(i, j, pixelArray);
                                    sRed += pixel[0];
                                    sGreen += pixel[1];
                                    sBlue += pixel[2];
                                }

                            int d = Distance(sRed / scale2, sGreen / scale2, sBlue / scale2, 0, 127, 127);
                            if (d < minD)
                                cube_color[x / scale][y / scale] = true;
                            else cube_color[x / scale][y / scale] = false;
                        }

                    imageWidth /= scale;
                    imageHeight /= scale;

                    int xCube = 0, yCube = 0, sMax = 0;
                    lee_matrix = new boolean[imageWidth][imageHeight];

                    for (int y = 0; y < imageHeight; ++y)
                        for (int x = 0; x < imageWidth; ++x) {
                            if (lee_matrix[x][y] == false && cube_color[x][y] == true) {
                                objS = 1;
                                // telemetry.update();
                                Lee(x, y);
                                //telemetry.update();
                                if (objS > sMax) {
                                    sMax = objS;
                                    xCube = x;
                                    yCube = y;
                                }
                            }
                        }

                    Loc = new int[3];
                    //telemetry.addData("Status", "NU");
                    //telemetry.update();
                    ObjectLee(xCube, yCube);
                    //telemetry.addData("Status", "DA");
                    // telemetry.update();
                    int locMax = Loc[left];
                    if (Loc[center] > locMax) locMax = Loc[center];
                    if (Loc[right] > locMax) locMax = Loc[right];
                    String S = "NONE";
                    if (Loc[left] == locMax) S = "LEFT";
                    else if (Loc[center] == locMax) S = "CENTER";
                    else if (Loc[right] == locMax) S = "RIGHT";

                    telemetry.addData("LOCATION", S);
                    telemetry.addData("Cube X", xCube * scale);
                    telemetry.addData("Cube Y", yCube * scale);
                    telemetry.addData("Cube Size", sMax * scale2);
                    telemetry.addData("Status", "End img processing: " + runtime.toString());

                    telemetry.update();
                                }
        }

    }

    public int Distance(int x1, int y1, int z1, int x2, int y2, int z2){

        int dx = x1 - x2, dy = y1 - y2, dz = z1 - z2;
        return dx * dx + dy * dy + dz * dz;
    }

    public int Max(int a, int b){
        if(a > b)return a;
        return b;
    }
    public int[] GetRGB(int x, int y, byte[] pixelArray) {

        int index = 3 * (y * imageWidth + x);
        int[] pixel = new int[5];
        pixel[3] = x;
        pixel[4] = y;
        for (int i = index; i <= index + 2; i++){
            if (pixelArray[i] < 0) pixelArray[i] += 128; //sometimes it gives negative values: -1 = 127, -2 = 126 etc
            pixel[i - index] = pixelArray[i];// * 2;  // * 2 is for comparison with 16 bit (0, 255) colors
        }
        return pixel;
    }

    public void ShowRGB(int[] pixel){
        telemetry.addData("X", pixel[3]);
        telemetry.addData("Y", pixel[4]);
        telemetry.addData("Red", pixel[0]);
        telemetry.addData("Green", pixel[1]);
        telemetry.addData("Blue", pixel[2]);
        telemetry.addLine();
    }

    public void Lee(int x, int y){

        int[] dx = {-1, 0, 1, -1, 1, -1, 0, 1};
        int[] dy = {-1, -1, -1, 0, 0, 1, 1, 1};
        int[][] Q = new int[imageWidth * imageHeight + 1][2];
        int st = 0, dr = 1;
        Q[st][0] = x;
        Q[st][1] = y;
        lee_matrix[x][y] = true;

        while(st < dr)
        {
            for (int d = 0; d < 8; ++d)
            {
                int x2 = Q[st][0] + dx[d], y2 = Q[st][1] + dy[d];
                if (Inside(x2, y2) && lee_matrix[x2][y2] == false && cube_color[x2][y2] == true)
                {
                    ++objS;
                    lee_matrix[x2][y2] = true;
                    Q[dr][0] = x2;
                    Q[dr][1] = y2;
                    ++dr;
                }
            }
            ++st;
        }

    }

    public void ObjectLee(int x, int y){

        int[] dx = {-1, 0, 1, -1, 1, -1, 0, 1};
        int[] dy = {-1, -1, -1, 0, 0, 1, 1, 1};
        int[][] Q = new int[imageWidth * imageHeight][2];
        int st = 0, dr = 1;
        Q[st][0] = x;
        Q[st][1] = y;
        lee_matrix[x][y] = false;

        while(st < dr)
        {
            CheckPos(Q[st][0]);
            for (int d = 0; d < 8; ++d)
            {
                int x2 = Q[st][0] + dx[d], y2 = Q[st][1] + dy[d];
                if (Inside(x2, y2) && lee_matrix[x2][y2] == true && cube_color[x2][y2] == true)
                {
                    lee_matrix[x2][y2] = false;
                    Q[dr][0] = x2;
                    Q[dr][1] = y2;
                    ++dr;
                }
            }
            ++st;
        }
    }

    public void CheckPos(int x){

        if(x < imageWidth / 3)++Loc[left];
        else if(x < 2 * imageWidth / 3)++Loc[center];
        else ++Loc[right];
    }

    public boolean Inside(int x, int y){

        return (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight);
    }
}


