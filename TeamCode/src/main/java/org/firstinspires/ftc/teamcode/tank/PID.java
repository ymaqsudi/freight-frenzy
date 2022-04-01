package org.firstinspires.ftc.teamcode.tank;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name="PID", group="Linear Opmode")
@Disabled
public class PID extends LinearOpMode {


    HardwarePushbot hardware = new HardwarePushbot();

    FtcDashboard dashboard;

    public static double TARGET_POS = 100;


    double integralSum = 0;
    public static double kP = .15;
    public static double kI = 0.01;
    public static double kD = 0.0;

    DcMotorEx left;
    DcMotorEx right;

    private BNO055IMU imu;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right.setDirection(DcMotorSimple.Direction.REVERSE);



        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        double referenceAngle = Math.toRadians(90);

        while(opModeIsActive()) {

            double power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle);
            right.setPower(power);
            left.setPower(-power);
        }
    }



    public double PIDControl (double reference, double state) {
        double error = angleWrap(reference - state);
        while (Math.abs(error) <= 10) {
            integralSum += error * timer.seconds();
            double derivative = (error - lastError) / timer.seconds();
            lastError = error;

            timer.reset();

            return (error * kP) + (derivative * kD) + (integralSum * kI);
        }

        return 0;

    }

    //
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

}
