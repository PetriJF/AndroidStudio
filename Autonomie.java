import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.atan;
import static java.lang.Math.toDegrees;


@Autonomous(name="Autonomie", group="Linear Opmode")
public class Autonomie extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private int imageWidth, imageHeight, scale = 8, left = 0, center = 1, right = 2;
    private boolean[][] cube_color, lee_matrix;
    private DcMotor motorLeft1 = null;
    private DcMotor motorRight1 = null;
    private DcMotor motorLeft2 = null;
    VuforiaLocalizer vuforia;
    private DcMotor motorRight2 = null;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private Object obj;
    private byte[] pixelArray;
    private VuforiaLocalizer.Parameters params = null;
    private VuforiaTrackables targetsRoverRuckus;
    private String S = "", nav = "";
    @Override
    public void runOpMode() throws InterruptedException {


        //tells VuforiaLocalizer to only store one frame at a time
        Init();
        waitForStart();
        runtime.reset();

        S = MineralsPhase();
        telemetry.log().add(S);

        nav = NavTargets();

        MoveMineral();
        // PlaceMarker();

    }

    public void Init()
    {


        params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AfpNx0X/////AAAAGZqo1zzaiEnYutr1KL/q14AabZMVkQIew0DVRrGP3F+HyPHj4YEvNmrsUBFPix2HcpgAbnD7aH+/KCxsYf/WWTS4q15H39Y2M9g5M4Wud6WNufbs5HsGaB8ipxMCDpWqhYv6KgAihfO3Zq3az4utKZ94CIuXATsi3oNABVks52lkFng/9WOM0LaroowGIMFyprWRUjmfggfaGlu/6mpGSBkvpNRlD8TRXYt/W/Qho4B7G03thikhfsdpNRoNCcZMRaIzWyo5oGWzslxyvdDp0bxBQB8tZsTW8p288zfXBGtSQ8pTXsgKpmjwdStIP0pXjDJ9av0+CXhJblIGvcmvm7K5DqANWk0Z6whToWD0+K7S";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        vuforia = ClassFactory.createVuforiaLocalizer(params);
        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        targetsRoverRuckus.get(0).setName("Blue");
        targetsRoverRuckus.get(1).setName("Red");
        targetsRoverRuckus.get(2).setName("Front");
        targetsRoverRuckus.get(3).setName("Back");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */


        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();
        telemetry.log().add("Gyro Calibrated!");


    }

    public String MineralsPhase() throws InterruptedException
    {
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        int ind = 1;
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();

        while (ind < 10)
        {
            frame = vuforia.getFrameQueue().take();
            ind++;
        }
        double whileStart = runtime.seconds();
        boolean ok = true;
        Object Cube = new Object();

        while(ok) {
            frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
            Image img = null;
            long numImages = frame.getNumImages();

            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888) {
                    img = frame.getImage(i);
                    if (img != null) {

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
                        FindColor(127, 90, 0);
                        pixelArray = null;

                         Cube = new Object();
                        lee_matrix = new boolean[imageWidth][imageHeight];

                        for (int y = 2 * imageHeight / 3; y < imageHeight; y++)
                            for (int x = 0; x < imageWidth; x++) {
                                if (!lee_matrix[x][y] && cube_color[x][y]) {
                                    obj = new Object();
                                    Lee(x, y);
                                    if (obj.size > Cube.size) Cube = obj;
                                }
                            }

                        String S;
                        if (Cube.size == 0) S = "NONE";
                        else S = Location(Cube.Loc);
                        telemetry.addData("LOCATION", S);
                        if (Cube.size != 0) {
                            telemetry.addData("Cube X", Cube.x * scale);
                            telemetry.addData("Cube Y", Cube.y * scale);
                            telemetry.addData("Cube Size", Cube.size * scale * scale);
                            double tanA = (double)(imageWidth / 2 - Cube.x) / (double)(imageHeight - Cube.y);
                            double angle = toDegrees(atan(tanA));
                            telemetry.addData("Angle", angle);
                        }

                        return Location(Cube.Loc);
                    }
                }
            }

        }
        return "NONE1";
    }

    public String NavTargets()
    {
        targetsRoverRuckus.activate();
        vuforia.setFrameQueueCapacity(30);

        rotateRobotRight(0.5f);
        sleep(200);
        setMotorSpeed(0f);
        telemetry.addData("heading ", modernRoboticsI2cGyro.getHeading());
        while (modernRoboticsI2cGyro.getHeading() > 330)
        {
            rotateRobotRight(0.15f);
            telemetry.addData("heading ", modernRoboticsI2cGyro.getHeading());
        }

        telemetry.update();
        setMotorSpeed(0f);
        boolean ok = true;
        while(ok)
            for (VuforiaTrackable trackable : targetsRoverRuckus) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();

                if (pose != null) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        return trackable.getName();
                    }
                }
            }

        return "NONE";
    }

    public void MoveMineral()
    {
        if (nav == "Front")
        {


            if (S == "LEFT")
            {
                rotateRobotLeft(0.15f);
                while (modernRoboticsI2cGyro.getHeading() < 360 && modernRoboticsI2cGyro.getHeading() > 180)
                    sleep(10);
                sleep(100);
                while (modernRoboticsI2cGyro.getHeading() < 20)
                    sleep(10);

                setMotorSpeed(0f);
            }

            else if (S == "CENTER")
            {
                rotateRobotLeft(0.15f);
                while (modernRoboticsI2cGyro.getHeading() < 345 && modernRoboticsI2cGyro.getHeading() > 180)
                    sleep(10);
                setMotorSpeed(0f);
            }

            else if (S == "RIGHT")
            {
                rotateRobotLeft(0.15f);
                while (modernRoboticsI2cGyro.getHeading() < 335 && modernRoboticsI2cGyro.getHeading() > 180)
                    sleep(10);
                setMotorSpeed(0f);
            }

            moveRobotForward(0.15f);
            sleep(500);

        }


    }

    private void PlaceMarker()
    {
        if(nav == "Front")
        {
            rotateRobotLeft(0.75f);
        }
    }

    private void moveRobotForward(float speed) {
        motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        setMotorSpeed(speed);
    }

    private void moveRobotReverse(float speed) {
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
        setMotorSpeed(speed);
    }

    private void rotateRobotRight(float speed) {
        motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
        setMotorSpeed(speed);
    }

    private void rotateRobotLeft(float speed) {
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        setMotorSpeed(speed);
    }


    private void setMotorSpeed(float speed) {
        motorLeft2.setPower(speed);
        motorRight2.setPower(speed);
        motorLeft1.setPower(speed);
        motorRight1.setPower(speed);
    }
    private void FindColor(int r, int g, int b){

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

                int d = Distance(sRed / scale2, sGreen / scale2, sBlue / scale2, r, g, b);
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

