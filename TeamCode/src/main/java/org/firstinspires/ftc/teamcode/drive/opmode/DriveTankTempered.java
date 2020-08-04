package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.TemperedTankDrive;

/**
 * This is a simple teleop routine for just driving the the robot around like a normal
 * tank drive teleop routine. It makes no attempt at localization.
 */
@Config
@TeleOp(name = "Tank Drive", group = "drive")
public class DriveTankTempered extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TemperedTankDrive robot = new TemperedTankDrive(hardwareMap);
        double leftPower = 0.0;
        double rightPower = 0.0;

        telemetry.update();
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            // Temper the gamepad controls for more delicate movements at low power.
            leftPower = robot.temper (gamepad1.left_stick_y); // etc.
            rightPower = robot.temper(gamepad1.right_stick_y);
            robot.setMotorPowers(leftPower, rightPower);
            robot.update();

            // Display the current value
            telemetry.addData("Time", "%5.3f", time);
            telemetry.addData(">", "Press Stop to end driving session." );

            telemetry.update();
        }
    }
}
