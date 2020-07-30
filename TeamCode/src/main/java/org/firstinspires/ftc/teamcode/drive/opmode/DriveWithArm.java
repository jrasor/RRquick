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
@TeleOp(name = "Drive and Use Arm", group = "drive")
public class DriveWithArm extends LinearOpMode {
    static final int    CYCLE_MS    =   10;    // period of each update cycle
    static final double STOWED      =  0.0;    // Retracted over robot body
    static final double DEPLOYED    =  0.8;    // Extended out over Field. 1.0 for HiTEKs.

    Servo arm;
    double position = STOWED;
    double minPosition = STOWED;
    double targetPosition;
    double startingPosition;
    double maxPosition = DEPLOYED;
    double maxPositionError = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        TemperedTankDrive robot = new TemperedTankDrive(hardwareMap);
        double leftPower = 0.0;
        double rightPower = 0.0;

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setPosition(STOWED);
//        Todo: see if this is of any use. arm.scaleRange(0,1.0);
        position = arm.getPosition();

        ElapsedTime runtime = new ElapsedTime();
        // Update frequency is 1 / timeScale.
        double timeScale               = 0.3;

        runtime.reset();
        double time                     = 0.0;        // Time since driving began.

        //  Arm range of motion will be full scale for now.
        double positionScale = DEPLOYED - STOWED;
        telemetry.addData(">", "Press Start to activate robot." );
        telemetry.update();
        waitForStart();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (opModeIsActive()) {
            // Temper the gamepad controls for more delicate movements at low power.
            leftPower = robot.temper (gamepad1.left_stick_y); // etc.
            rightPower = robot.temper(gamepad1.right_stick_y);
            robot.setMotorPowers(leftPower, rightPower);
            robot.update();

            if (gamepad1.a) {
                runtime.reset();
                minPosition = targetPosition = STOWED;
                maxPosition = startingPosition = position = arm.getPosition();
                positionScale = minPosition - maxPosition; // yes, negative.
            }
            if (gamepad1.y) {
                runtime.reset();
                maxPosition = targetPosition = DEPLOYED;
                minPosition = startingPosition = position = arm.getPosition();
                positionScale = maxPosition - minPosition;
            }

            // Display the current value
            telemetry.addData("Time", "%5.3f", time);
            telemetry.addData("Arm Position", "%5.2f", position);
            telemetry.addData("Starting Position", "%5.2f", startingPosition);
            telemetry.addData("Target Position", "%5.2f", targetPosition);
            telemetry.addData(">", "Press Stop to end driving session." );

            // update time and positions
            time = runtime.time();
            if (Math.abs (targetPosition - position) > maxPositionError) {
                position = startingPosition + positionScale * (0.5 - 0.5 * Math.cos(timeScale * Math.PI * time));
                arm.setPosition(position);
                telemetry.addData ("", "Arm moving...");
            }
            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }
    }
}
