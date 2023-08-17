package org.firstinspires.ftc.teamcode.OpenCv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous

public class Auto extends LinearOpMode{

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;



    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lb");
        rightRear = hardwareMap.get(DcMotorEx.class, "rb");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");

        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        while (!isStarted() && !isStopRequested())
        {

        }

        while (opModeIsActive()){

            leftRear.setPower(0.3);
            rightFront.setPower(0.3);
            leftFront.setPower(0.3);
            rightRear.setPower(0.3);
            if(isStopRequested()){
                return;
        }

    }
}
}



