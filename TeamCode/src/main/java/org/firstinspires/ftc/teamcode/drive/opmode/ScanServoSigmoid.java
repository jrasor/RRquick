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

import static java.lang.Math.round;

/**
 *      This OpMode continuously scans a single servo back and forwards until Stop is pressed,
 * following a sigmoid angle vs time function over the interval (0, 1):
 *
 *      r (t) = 0.5 - 0.5 * cos (πt)
 *
 *      This function starts slowly from zero, builds up some speed until the halfway
 *  point is reached, then slows down to gently approach the endpoint.
 *      The domain and range can be scaled.
 *
 *      The code is structured as a LinearOpMode.
 *      CYCLE_MS sets the update period.
 *
 *      This code assumes a servo with the name "arm" as found on a Trainerbot.
 *
 *      NOTE: When any servo position is set, ALL attached servos are activated, so ensure
 * that any other connected servos are able to move freely before running this test.
 */

@TeleOp(name = "Scan Arm Gently", group = "Actuators")
//@Disabled
public class ScanServoSigmoid extends LinearOpMode {
    static final int    CYCLE_MS    =   50;     // period of each report cycle
    static final double STOWED      =  0.00;     // Retracted over robot body
    static final double DEPLOYED      =  0.80;     // Retracted over robot body
    static final double HALFWAY     =  (DEPLOYED - STOWED)/2;
//    double time                     = 0.0;        // Time since test began.
    // At scale 1.0, 1.0 second between extreme positions STOWED and DEPLOYED.
    double timeScale                     = 0.1;
    double positionScale = Math.abs (DEPLOYED - STOWED);

    // Define class members
    Servo armServo;
    double position = STOWED;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        // Change the text in quotes to match a servo name on your robot.
        armServo = hardwareMap.get(Servo.class, "arm");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan arm." );
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            armServo.setPosition(position);

            //  Report on time and position
            telemetry.addData("Run time", "%5.3f", time);
            telemetry.addData("Servo position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // update time and positions
            time = runtime.time();
            position = positionScale * (0.5 - 0.5 * Math.cos(timeScale * Math.PI * time));
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
