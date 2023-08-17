package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class GetCookies {
    public DcMotorEx gl = null;
    public DcMotorEx gr = null;

    private PIDController controllerl;
    private PIDController controllerr;

    public ElapsedTime runtime = new ElapsedTime();

    public static double maxPIDPower = 0.5;
    public static double kP = 0.013;
    public static double kI = 0.0001;
    public static double kD = 0.0001;
    public static double kF = 0.003;

    public static double setpoint = 0;

    double ticks_in_degrees = 145.1 / 360;

    public GetCookies(HardwareMap hardwareMap) {

        setpoint = 0;

        gl = hardwareMap.get(DcMotorEx.class, "gl");
        gr = hardwareMap.get(DcMotorEx.class, "gr");

        gl.setDirection(DcMotorSimple.Direction.REVERSE);

        gl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        gr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        gl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        gr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        gl.setPower(0);
        gr.setPower(0);

        controllerl = new PIDController(kP,kI,kD);
        controllerr = new PIDController(kP,kI,kD);
    }

    public void update(){

        if(setpoint == 0 && gl.getCurrentPosition() < 50 && gr.getCurrentPosition() < 50){
            gl.setPower(0);
            gr.setPower(0);
        }else {
            controllerl.setPID(kP, kI, kD);
            int glPos = gl.getCurrentPosition();
            double pidl = Math.min(controllerl.calculate(glPos, setpoint), maxPIDPower);
            double ffl = Math.cos(Math.toRadians(setpoint / ticks_in_degrees)) * kF;

            double powerl = pidl + ffl;

            gl.setPower(powerl);


            controllerr.setPID(kP, kI, kD);
            int grPos = gr.getCurrentPosition();
            double pidr = Math.min(controllerr.calculate(grPos, setpoint), maxPIDPower);
            double ffr = Math.cos(Math.toRadians(setpoint / ticks_in_degrees)) * kF;

            double powerr = pidr + ffr;

            gr.setPower(powerr);


        }

    }


    public void down(){
        setpoint = 0;
    }

    public void up(int level){
        if(level == 1){
            setpoint = 300;
        }else if(level == 2){
            setpoint = 730;
        }else{
            setpoint = 0;
        }
    }

}
