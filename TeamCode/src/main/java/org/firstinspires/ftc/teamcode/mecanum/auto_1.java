package org.firstinspires.ftc.teamcode.mecanum;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name="auto 1", group="Linear Opmode")
public class auto_1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    hardware_mecanum hardware = new hardware_mecanum();
    auto_test_methods methods = new auto_test_methods(hardware);


    @Override
    public void runOpMode() {

        hardware.init(hardwareMap);




        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();



        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            methods.driveForwardOrBackward(10,5);
            stop();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }


    }





}

