package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    private int imageWidth, imageHeight, scale = 8, left = 0, center = 1, right = 2;
    private boolean[][] cube_color, lee_matrix;
    private Object obj;
    private byte[] pixelArray;
    @Override
    public void runOpMode() throws InterruptedException {


        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AfpNx0X/////AAAAGZqo1zzaiEnYutr1KL/q14AabZMVkQIew0DVRrGP3F+HyPHj4YEvNmrsUBFPix2HcpgAbnD7aH+/KCxsYf/WWTS4q15H39Y2M9g5M4Wud6WNufbs5HsGaB8ipxMCDpWqhYv6KgAihfO3Zq3az4utKZ94CIuXATsi3oNABVks52lkFng/9WOM0LaroowGIMFyprWRUjmfggfaGlu/6mpGSBkvpNRlD8TRXYt/W/Qho4B7G03thikhfsdpNRoNCcZMRaIzWyo5oGWzslxyvdDp0bxBQB8tZsTW8p288zfXBGtSQ8pTXsgKpmjwdStIP0pXjDJ9av0+CXhJblIGvcmvm7K5DqANWk0Z6whToWD0+K7S";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        locale.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time


        waitForStart();
        runtime.reset();
        while (opModeIsActive())
        {

            double whileStart = runtime.seconds();
            VuforiaLocalizer.CloseableFrame frame = locale.getFrameQueue().take(); //takes the frame at the head of the queue
            Image img = null;
            long numImages = frame.getNumImages();

            for (int i = 0; i < numImages; i++)
            {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888)
                {
                    img = frame.getImage(i);
                    break;
                }
            }

            if (img != null)
            {

                telemetry.addData("Status", "Start img processing: " + runtime.toString());
                ByteBuffer pixels = img.getPixels();
                pixelArray = new byte[pixels.remaining()];
                pixels.get(pixelArray, 0, pixelArray.length);
                pixels = null;
                imageWidth = img.getWidth();
                imageHeight = img.getHeight();
                /*int stride = img.getStride();
                  telemetry.addData("Image", "Image width: " + imageWidth);
                  telemetry.addData("Image", "Image height: " + imageHeight);
                  telemetry.addData("Image", "Image stride: " + stride);
                  telemetry.addData("Image", "First pixel byte: " + pixelArray[0]);
                  telemetry.addData("Length", pixelArray.length);
                  0, 0 right up
                  imageWidth - 1, imageHeight  - 1 left down*/

                cube_color = new boolean[imageWidth / scale][imageHeight / scale];
                FindColor();
                pixelArray = null;

                Object Cube = new Object();
                lee_matrix = new boolean[imageWidth][imageHeight];

                for (int y = 0; y < imageHeight; y++)
                    for (int x = 0; x < imageWidth; x++)
                    {
                        if (!lee_matrix[x][y] && cube_color[x][y])
                        {
                            obj = new Object();
                            Lee(x, y);
                            if (obj.size > Cube.size)Cube = obj;
                        }
                    }

                String S;
                if(Cube.size == 0)S = "NONE";
                else S = Location(Cube.Loc);
                telemetry.addData("LOCATION", S);
                if(Cube.size != 0)
                {
                    telemetry.addData("Cube X", Cube.x * scale);
                    telemetry.addData("Cube Y", Cube.y * scale);
                    telemetry.addData("Cube Size", Cube.size * scale * scale);
                }
                telemetry.addData("Status", "End img processing: " + runtime.toString());
            }

            double whileTime = runtime.seconds() - whileStart;
            whileTime = (double)((int)(whileTime * 100)) / 100;
            telemetry.addData("Time for one while loop", whileTime);
            telemetry.update();
        }

    }

    private void FindColor(){

        int scale2 = scale * scale, minD = 5000;//minD = minimal difference between 2 colors for comparision
        for (int y = 0; y < imageHeight - scale; y += scale)
            for (int x = 0; x < imageWidth - scale; x += scale)
            {

                int[] pixel;
                int sRed = 0;
                int sGreen = 0;
                int sBlue = 0;

                for (int j = y; j < y + scale; j++)
                    for (int i = x; i < x + scale; i++)
                    {
                        pixel = GetRGB(i, j);
                        sRed += pixel[0];
                        sGreen += pixel[1];
                        sBlue += pixel[2];
                    }

                int d = Distance(sRed / scale2, sGreen / scale2, sBlue / scale2, 127, 127, 0);
                if(d < minD)
                    cube_color[x / scale][y / scale] = true;
            }

        imageWidth /= scale;
        imageHeight /= scale;

    }

    private int[] GetRGB(int x, int y){

        int index = 3 * (y * imageWidth + x);
        int[] pixel = new int[3];
        for (int i = index; i <= index + 2; i++)
        {
            if (pixelArray[i] < 0)
                pixelArray[i] += 128; //sometimes it gives negative values: -1 = 127, -2 = 126 etc
            pixel[i - index] = pixelArray[i];
        }
        return pixel;
    }

    private int Distance(int x1, int y1, int z1, int x2, int y2, int z2){

        int dx = x1 - x2, dy = y1 - y2, dz = z1 - z2;
        return dx * dx + dy * dy + dz * dz;
    }

    class Object{
        int x, y, size;
        int[] Loc = new int[3];
        private Object(){}
    }

    private void Lee(int x, int y){

        int[] dx = {-1, 0, 1, -1, 1, -1, 0, 1};
        int[] dy = {-1, -1, -1, 0, 0, 1, 1, 1};
        int[][] Q = new int[imageWidth * imageHeight + 1][2];
        int st = 0, dr = 1;

        obj.x = x;
        obj.y = y;

        Q[st][0] = x;
        Q[st][1] = y;
        lee_matrix[x][y] = true;

        while(st < dr)
        {
            ++obj.size;
            CheckPos(Q[st][0]);
            for (int d = 0; d < 8; d++)
            {
                int x2 = Q[st][0] + dx[d], y2 = Q[st][1] + dy[d];
                if (Inside(x2, y2) && !lee_matrix[x2][y2] && cube_color[x2][y2])
                {
                    lee_matrix[x2][y2] = true;
                    Q[dr][0] = x2;
                    Q[dr][1] = y2;
                    ++dr;
                }
            }
            ++st;
        }

    }

    private void CheckPos(int x){

        if(x < imageWidth / 3)++obj.Loc[left];
        else if(x < 2 * imageWidth / 3)++obj.Loc[center];
        else ++obj.Loc[right];
    }

    private boolean Inside(int x, int y){

        return (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight);
    }

    private String Location(int[] Loc){

        int locMax = Loc[left];
        if (Loc[center] > locMax)
            locMax = Loc[center];
        if (Loc[right] > locMax)
            locMax = Loc[right];

        if (Loc[left] == locMax) return "LEFT";
        if (Loc[center] == locMax) return "CENTER";
        if (Loc[right] == locMax) return "RIGHT";

        return "?";
    }
}
