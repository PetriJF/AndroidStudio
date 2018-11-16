import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Rotatie Autonomie", group="Linear Opmode")
public class RotatieAutonomie extends LinearOpMode {

    private Robot robot;
    private DcMotor motorLeft1 = null;
    private DcMotor motorRight1 = null;
    private DcMotor motorLeft2 = null;
    private DcMotor motorRight2 = null;

    @Override
    public void runOpMode() throws InterruptedException
    {

        Init();
        waitForStart();

        robot.rotate(360);
        robot.rotate(-360);
    }

    public void Init()
    {

        robot = new Robot();
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");

        telemetry.log().add("Init completed");

    }

    public class Robot
    {

        private Robot(){}

        public void rotate(int degrees)
        {

            int ticks = toTicks(degrees);

            motorLeft1.setDirection(DcMotor.Direction.FORWARD);
            motorLeft2.setDirection(DcMotor.Direction.FORWARD);
            motorRight1.setDirection(DcMotor.Direction.FORWARD);
            motorRight2.setDirection(DcMotor.Direction.FORWARD);
            motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setMotorSpeed(ticks > 0 ? 0.4f : -0.4f);

            int startPos = Math.abs(motorRight1.getCurrentPosition());
            while (Math.abs(motorRight1.getCurrentPosition()) < Math.abs(ticks) + startPos)
            {
                telemetry.addData("Ticks", motorRight1.getCurrentPosition());
                telemetry.addData("Goal", ticks);
                telemetry.update();
            }

            setMotorSpeed(0f);
            motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(350);

        }

    }

    public int toTicks(int degrees)
    {
            int maxTick = degrees >= 0 ? 2800 : 1700;
            return maxTick * degrees / 360;
    }

    private void setMotorSpeed(float speed)
    {
        motorLeft2.setPower(speed);
        motorRight2.setPower(speed);
        motorLeft1.setPower(speed);
        motorRight1.setPower(speed);
    }
}
