package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp1")
public class TeleOP1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        RobotHardware robot = new RobotHardware(hardwareMap);
        runtime.reset();
        waitForStart();
        while (opModeIsActive()) {
            robot.DriveMovement(gamepad1);
            telemetry.addData("Runtime Seconds - ", runtime.seconds());


            telemetry.update();

        }
    }
}

