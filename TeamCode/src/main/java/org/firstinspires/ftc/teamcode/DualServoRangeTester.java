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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImpl;

@TeleOp(name = "Dual Servo Range Tester (Dashboard Enabled)", group = "Tester")
@Config
public class DualServoRangeTester extends OpMode {

    public static double aPos1 = 0.5;
    public static double bPos1 = 0.5;
    public static double xPos1 = 0.5;
    public static double yPos1 = 0.5;
    public static double lPos1 = 0.5;
    public static double rPos1 = 0.5;
    public static double aPos2 = 0.5;
    public static double bPos2 = 0.5;
    public static double xPos2 = 0.5;
    public static double yPos2 = 0.5;
    public static double lPos2 = 0.5;
    public static double rPos2 = 0.5;

    private ServoImpl testServo1;
    private ServoImpl testServo2;

    @Override
    public void init() {
        testServo1 = hardwareMap.get(ServoImpl.class, "TestServo1");
        testServo2 = hardwareMap.get(ServoImpl.class, "TestServo2");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            testServo1.setPosition(aPos1);
            testServo2.setPosition(aPos2);
        } else if (gamepad1.b) {
            testServo1.setPosition(bPos1);
            testServo2.setPosition(bPos2);
        } else if (gamepad1.x) {
            testServo1.setPosition(xPos1);
            testServo2.setPosition(xPos2);
        } else if (gamepad1.y) {
            testServo1.setPosition(yPos1);
            testServo2.setPosition(yPos2);
        } else if (gamepad1.left_bumper) {
            testServo1.setPosition(lPos1);
            testServo2.setPosition(lPos2);
        } else if (gamepad1.right_bumper) {
            testServo1.setPosition(rPos1);
            testServo2.setPosition(rPos2);
        }
    }
}
