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

import android.content.res.Resources;

import  com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.Arm;
import org.firstinspires.ftc.teamcode.classes.GetCookies;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Doamne Miluieste", group="Linear Opmode")

public class OfficialOP extends LinearOpMode {

    SampleMecanumDrive cookie;
    MiniCookies minicookies;
    GetCookies lift;

    Arm putcookies;

    double speedM = 0.85f;
    boolean ok_speed = false;

    boolean highj = false;
    boolean midj = false;
    boolean base = false;

    @Override
    public void runOpMode() {

        putcookies = new Arm(hardwareMap);
        cookie = new SampleMecanumDrive(hardwareMap);
        minicookies = new MiniCookies(hardwareMap);
        lift = new GetCookies(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        final Thread high = new Thread() {
            public void run() {
                lift.up(2);
                minicookies.pick.setPosition(0.50 );
                //minicookies.put();
            }
        };
        final Thread down_high = new Thread() {
            public void run() {
                minicookies.take();
                OfficialOP.this.sleep(250);
                lift.down();
                minicookies.pick.setPosition(0.1);
            }
        };
        final Thread mid = new Thread() {
            public void run() {
                lift.up(1);
                minicookies.pick.setPosition(0.54);
                //minicookies.put();
            }
        };
        final Thread down_mid = new Thread() {
            public void run() {
                minicookies.take();
                OfficialOP.this.sleep(250);
                lift.down();
                OfficialOP.this.sleep(250);
                minicookies.pick.setPosition(0.1);
            }
        };
        final Thread arm_up = new Thread() {
            public void run() {
                minicookies.close();
                OfficialOP.this.sleep(150);
                if (base) {
                    minicookies.def();
                    base = false;
                }
                OfficialOP.this.sleep(150);
                minicookies.up();
                putcookies.up_arm();
            }
        };
        final Thread tostart = new Thread() {
            public void run() {
                putcookies.up_arm_to_pos(120);
                OfficialOP.this.sleep(1000);
                minicookies.startoff();
            }
        };

        minicookies.init();

        waitForStart();

        tostart.start();
        while (tostart.isAlive()) {
            putcookies.update();
        }

        //after start;

        while (opModeIsActive()) {

            lift.update();
            putcookies.update();

            if (gamepad1.cross && !ok_speed) {
                ok_speed = true;
                if (speedM == 0.85f) {
                    speedM = 2f;
                } else {
                    speedM = 0.9f;
                }
            }

            if (!gamepad1.cross) {
                ok_speed = false;
            }

            double y = -gamepad1.left_stick_y * 0.7;
            double x = gamepad1.left_stick_x * 1.3;
            double rx = gamepad1.right_stick_x;

            cookie.setWeightedDrivePower(new Pose2d(y * speedM,
                    -x * 0.3,
                    -rx * speedM));

            //༼ つ ◕_◕ ༽つ🍪
            // Gamepad 2

            //Ground
            if (gamepad2.square) {
                minicookies.open();
            }
            if (gamepad2.circle) {
                minicookies.close();
            }
            //Arm up/down
            if (gamepad2.triangle) {
                arm_up.start();
            }
            if (gamepad2.cross) {
                putcookies.down_arm();
                minicookies.down();
            }
            //Lift
            //down
            if (gamepad2.dpad_down) {
                if (midj) {
                    midj = false;
                    minicookies.take();
                    down_mid.start();
                } else if (highj) {
                    highj = false;
                    down_high.start();
                }
                speedM = 0.9f;
            }
            //mid
            if (gamepad2.dpad_right) {
                midj = true;
                mid.start();
                speedM = 0.65f;
            }
            //high
            if (gamepad2.dpad_up) {
                highj = true;
                high.start();
                speedM = 0.65f;
            }
            //Bumper
            if (gamepad1.right_bumper) {
                minicookies.put();
            }
            if (gamepad1.left_bumper) {
                minicookies.take();
            }

            minicookies.pick.setPosition(minicookies.pick.getPosition() + gamepad2.right_stick_x / 100);

            minicookies.posa.setPosition(minicookies.posa.getPosition() + gamepad2.right_stick_y / 100);

            telemetry.addData("rotatie", minicookies.base.getCurrentPosition());
            telemetry.addData("pozarm", minicookies.posa.getPosition());
            telemetry.addData("picki", minicookies.pick.getPosition());
            telemetry.addData("gl_st", lift.gl.getCurrentPosition());
            telemetry.addData("gl_rg", lift.gr.getCurrentPosition());
            telemetry.addData("target", putcookies.setpoint_arm);
            telemetry.addData("position", putcookies.arm.getCurrentPosition());
            telemetry.update();
        }
    }
}

