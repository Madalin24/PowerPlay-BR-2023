/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCv;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.classes.Arm;
import org.firstinspires.ftc.teamcode.classes.GetCookies;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class Blue extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 0;
    int MIDDLE = 4;
    int RIGHT = 2;


    AprilTagDetection tagOfInterest = null;

    SampleMecanumDrive drive;
    MiniCookies minicookies;
    GetCookies lift ;
    Arm arm ;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    double drivee;   // Power for forward and back motion
    double strafe;  // Power for left and right motion
    double rotate;  // Power for rotating the robot

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime start = new ElapsedTime();


    @Override
    public void runOpMode()
    {


//        drive = new SampleMecanumDrive(hardwareMap);
        minicookies = new MiniCookies(hardwareMap);
        lift = new GetCookies(hardwareMap);
        arm = new Arm(hardwareMap);


        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lb");
        rightRear = hardwareMap.get(DcMotorEx.class, "rb");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);


/*
        TrajectorySequence tr1 = drive.trajectorySequenceBuilder(new Pose2d(31.45, -63.28, Math.toRadians(90.00)))
                .splineTo(new Vector2d(37.89, -18.38), Math.toRadians(94.04))
                .addTemporalMarker(0,()->{
                    minicookies.startoff();
                })
                .splineToLinearHeading(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)), Math.toRadians(23.79))
                .build();


        TrajectorySequence tr1 = drive.trajectorySequenceBuilder(new Pose2d(31.45, -63.28, Math.toRadians(90.00)))
                .lineTo(new Vector2d(36.05, -61.21))
                .addTemporalMarker(0,()->{
                    minicookies.startoff();
                })
                .splineTo(new Vector2d(37.89, -18.38), Math.toRadians(89.37))
                .splineToLinearHeading(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)), Math.toRadians(23.69))
                .build();




        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)))
                .lineToLinearHeading(new Pose2d(35.29, -11.83, Math.toRadians(-90.00)))
                .lineToLinearHeading(new Pose2d(11.42, -12.97, Math.toRadians(-90.00)))
                .build();




        TrajectorySequence mid = drive.trajectorySequenceBuilder(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)))
                .lineToLinearHeading(new Pose2d(35.29, -11.83, Math.toRadians(-90.00)))
                .build();



        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)))
                .lineToLinearHeading(new Pose2d(60.43, -12.95, Math.toRadians(-90.00)))
                .build();

    drive.setPoseEstimate(tr1.start());

    */






        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("CAMERA VEDE ALA!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("nu vede ala :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){


            driveForward(0.7);
            sleep(550);
            turnLeft(0.7);
            sleep(400);
            driveForward(0.69);
            sleep(325);
            stopDriving();
            arm.up_arm_to_pos(0);
            minicookies.init();
/*
            driveForward(0.7);
            sleep(1069);
            turnRight(0.6);
            sleep(469);
            driveForward(0.5);
            sleep(250);
            stopDriving();
       /*     turnLeft(0.4);
            sleep(750);
            driveForward(0.4);
            sleep(1060);
            stopDriving();*/

       /*     driveForward(0.4);.
            sleep(2500);
            turnRight(0.4);

            sleep(1500);
            //codul de brat + glisiere de 3-5 ori loop

            turnRight(0.4);
            sleep(1000);
            driveForward(0.4);
            sleep(1250);
            stopDriving();

*/

        }else if(tagOfInterest.id == MIDDLE){


            driveForward(0.5);
            sleep(630) ;
            stopDriving();
            arm.up_arm_to_pos(0);
            minicookies.init();
        /*
            driveForward(0.4);
            sleep(2500);
            turnRight(0.4);
            sleep(1400);
            //codul de brat + glisiere de 3-5 ori loop

            stopDriving();
*/
        } else if (tagOfInterest.id == RIGHT) {

            driveForward(0.7);
            sleep(500);
            turnRight(0.7);
            sleep(345);
            driveForward(0.69);
            sleep(250);
            stopDriving();
            arm.up_arm_to_pos(0);
            minicookies.init();
        }

    }

    private void driveForward(double power){
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        rightFront.setPower(power);
    }
    private void turnRight(double power){
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        leftFront.setPower(power);
        leftRear.setPower(power);
    }
    private void turnLeft(double power){
        leftFront.setPower(-power);
        leftRear.setPower(-power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }
    private void stopDriving(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Trans  lation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}

