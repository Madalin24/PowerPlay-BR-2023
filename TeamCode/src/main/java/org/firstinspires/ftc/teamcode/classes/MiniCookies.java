package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@Config
public class MiniCookies {

    public ServoImplEx upl = null;
    public ServoImplEx upr = null;
    public ServoImplEx posa = null;
    public ServoImplEx claw = null;
    public DcMotorEx base = null;

    private ElapsedTime runtime = new ElapsedTime();

    public MiniCookies(HardwareMap hardwareMap){

        base = hardwareMap.get(DcMotorEx.class, "base");

        posa = hardwareMap.get(ServoImplEx.class, "posa");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        upl = hardwareMap.get(ServoImplEx.class, "upl");
        upr = hardwareMap.get(ServoImplEx.class, "upr");


        base.setDirection(DcMotorEx.Direction.REVERSE);
        upr.setDirection(com.qualcomm.robotcore.hardware.ServoImplEx.Direction.REVERSE);
        base.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        base.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        base.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        poweroff();

        base.setTargetPosition(0);
    }


    public void poweroff(){
        upl.setPwmDisable();
        upr.setPwmDisable();

        posa.setPwmDisable();
        claw.setPwmDisable();
    }

    public void init() {

        runtime.reset();
        while (runtime.seconds() < 1) {
            posa.setPosition(0);

            claw.setPosition(0.20);

            if (runtime.seconds() < 1.2) {
                upl.setPosition(0);
                upr.setPosition(0);
            }
        }
    }

    public void take(){
        upl.setPosition(0.30);
        upr.setPosition(0.30);
    }

    public void put(){
        upl.setPosition(0.8);
        upr.setPosition(0.8);
    }

    public void open(){
        claw.setPosition(0.560);
    }
    public void close(){
        claw.setPosition(0.450);
    }

    //tuning
    //
    /*public void load(){

        close();
        runtime.reset();

        while(runtime.seconds()<1.25){
            if(runtime.seconds() > 0.25){
                arm1.setPosition(0.1772);
                arm2.setPosition(0.1772);
            }
            if(runtime.seconds()>0.45){
                posa.setPosition(0.243);

            }
            if(runtime.seconds()>0.75){
                base.setTargetPosition(0);
                base.setPower(1);
            }

            if(runtime.seconds() > 1.20){
                open();
            }
        }

        posa.setPosition(0.241);

        //arm1.setPosition(0.307);
        //arm2.setPosition(0.307);


    }*/




    /*public void stack5(){
        open();
        posa.setPosition(0.57);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-544);
                base.setPower(1);
            }
        }



        arm1.setPosition(0.488);
        arm2.setPosition(0.488);




    }

    public void stack4(){

        open();
        posa.setPosition(0.5683);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-544);
                base.setPower(1);
            }
        }


        arm1.setPosition(0.502);
        arm2.setPosition(0.502);

    }

    public void stack3(){

        open();
        posa.setPosition(0.5294);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-544);
                base.setPower(1);
            }
        }

        arm1.setPosition(0.5183);
        arm2.setPosition(0.5183);



    }

    public void stack2(){

        open();
        posa.setPosition(0.527);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-544);
                base.setPower(1);
            }
        }
        arm1.setPosition(0.542);
        arm2.setPosition(0.542);


    }

    public void stack1(){

        open();

        posa.setPosition(0.5);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-544);
                base.setPower(1);
            }
        }

        arm1.setPosition(0.563);
        arm2.setPosition(0.563);

    }*/

}
