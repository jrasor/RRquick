package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.TemperedTankDrive;

/**
 * This is a simple teleop routine for just driving the the robot around like a normal
 * tank drive teleop routine. It makes no attempt at localization.
 */
@Config
@TeleOp(group = "drive")
public class TankDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TemperedTankDrive drive = new TemperedTankDrive(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            drive.setMotorPowers(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            drive.update();
            telemetry.update();
        }
    }
}
