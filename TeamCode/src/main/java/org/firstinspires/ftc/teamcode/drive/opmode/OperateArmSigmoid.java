/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "paddle" as is found on a Trainerbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 * ToDo: remove that constraint.
 */

@TeleOp(name = "Operate Arm Gently", group = "Actuators")
//@Disabled
public class OperateArmSigmoid extends LinearOpMode {

    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   10;     // period of each up date cycle
    static final double STOWED      =  0.0;     // Retracted over robot body
    static final double DEPLOYED    =  0.80;     // Extended out over Field
    static final double HALFWAY    =   (DEPLOYED - STOWED)/2;

    // Define class members
    Servo   arm;
    double position; // = arm.getPosition(); // STOWED;
    double minPosition = STOWED;
    double targetPosition; //  = STOWED;
    double startingPosition;
    double maxPosition = DEPLOYED;
    double maxPositionError = 0.01;

    @Override
    public void runOpMode() {
        //  Initialize servo and arm.
        arm = hardwareMap.get(Servo.class, "arm");
//        arm.setPosition(STOWED);
        arm.setPosition(HALFWAY);
//        arm.scaleRange(0,1.0);
        position = arm.getPosition();
        telemetry.addData("Arm starting at", "%5.2f", position);
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();
        // At timescale 0.5, 2.0 seconds between extreme positions STOWED and DEPLOYED.
        double timeScale               = 0.3;

        // Wait for the start button
        //telemetry.addData(">", "Press Start to activate arm." );
        //telemetry.update();
        waitForStart();
        runtime.reset();
        double time                     = 0.0;        // Time since test began.

        //  At positionScale 1.0, range is fully DEPLOYED - STOWED.
        double positionScale = DEPLOYED - STOWED;
        while(opModeIsActive()){
                if (gamepad1.a) { // ** goes wrong way, over 1
                    runtime.reset();
                    minPosition = targetPosition = STOWED;
                    maxPosition = startingPosition = position = arm.getPosition();
                    positionScale = minPosition - maxPosition;
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
            telemetry.addData(">", "Press Stop to end test." );

            // update time and positions
            time = runtime.time();
            if (Math.abs (targetPosition - position) > maxPositionError) {
                position = startingPosition + positionScale * (0.5 - 0.5 * Math.cos(timeScale * Math.PI * time));
                arm.setPosition(position);
                telemetry.addData ("", "Moving...");
            }
            telemetry.update();
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
