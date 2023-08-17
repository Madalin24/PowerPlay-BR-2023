package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Arm {
    public DcMotorEx arm = null;

    private PIDController controller;

    public static double maxPIDPower = 1;
    public static double kP = 0.004;
    public static double kI = 0;

    public static double kD = 0.00054;
    public static double kF = 0.08;
    public static int offset = 460;

    public static double setpoint_arm = 0;

    public double  ticks_in_degrees = 1153.6193 / 360;

    boolean ok = false;

    public Arm(HardwareMap hardwareMap) {

        setpoint_arm = 0;

        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        arm.setPower(0);

        controller = new PIDController(kP,kI,kD);
    }

    public void update(){


        controller.setPID(kP, kI, kD);
        int armPos = arm.getCurrentPosition();
        double pid = Math.min(controller.calculate(armPos, setpoint_arm), maxPIDPower);
        double ffl = Math.cos(Math.toRadians((setpoint_arm - offset) / ticks_in_degrees)) * -kF;

        double power = pid + ffl;

        arm.setPower(power);


    }

    public void modify(){
        offset = 0;
        ok = true;
    }

    public void up_arm(){

        if(ok){
            setpoint_arm = -355;
        }else{
            setpoint_arm = 100;
        }
    }

    public void down_arm(){
        if(ok){
            setpoint_arm = -35;
        }else {
            setpoint_arm = 441;
        }
    }

    public void up_arm_to_pos(double postion){
        setpoint_arm = postion;
    }
}
