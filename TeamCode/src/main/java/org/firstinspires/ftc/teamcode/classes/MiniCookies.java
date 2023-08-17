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

    Arm arm;

    public ServoImplEx upl = null;
    public ServoImplEx upr = null;
    public ServoImplEx posa = null;
    public ServoImplEx claw = null;
    public ServoImplEx pick = null;
    public DcMotorEx base = null;

    private ElapsedTime runtime = new ElapsedTime();

    public MiniCookies(HardwareMap hardwareMap){

        arm = new Arm(hardwareMap);

        base = hardwareMap.get(DcMotorEx.class, "base");

        posa = hardwareMap.get(ServoImplEx.class, "posa");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        upl = hardwareMap.get(ServoImplEx.class, "upl");
        upr = hardwareMap.get(ServoImplEx.class, "upr");

        pick = hardwareMap.get(ServoImplEx.class, "pick");


        base.setDirection(DcMotorEx.Direction.REVERSE);
        upr.setDirection(com.qualcomm.robotcore.hardware.ServoImplEx.Direction.REVERSE);
        pick.setDirection(com.qualcomm.robotcore.hardware.ServoImplEx.Direction.REVERSE);
        base.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        base.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        base.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pick.setPosition(0);

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
            posa.setPosition(1);

            claw.setPosition(0.30);
            pick.setPosition(0);

            if (runtime.seconds() < 1.2) {
                upl.setPosition(0);
                upr.setPosition(0);
            }
        }
    }

    public void take(){
        upl.setPosition(0.14);
        upr.setPosition(0.14);
    }

    public void put(){
        upl.setPosition(0.75);
        upr.setPosition(0.75);
    }

    public void open(){
        claw.setPosition(0.50);
    }
    public void close(){
        claw.setPosition(0.70);
    }
    public void down() {
        posa.setPosition(0.42);
    }
    public void up() { posa.setPosition(1); }
    public void def() {
        base.setPower(0.6);
        base.setTargetPosition(0);
    }
    public void startoff() {
        runtime.reset();
        while (runtime.seconds() < 1) {
            up();
            open();

            pick.setPosition(0);

            if (runtime.seconds() < 1.2) {
                upl.setPosition(0.14);
                upr.setPosition(0.14);
            }
        }
    }

    public void stack5(){
        open();
        posa.setPosition(0.57);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-350);
                base.setPower(1);
            }
        }
        arm.up_arm_to_pos(441);
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
        arm.up_arm_to_pos(430);
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
        arm.up_arm_to_pos(400);
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
        arm.up_arm_to_pos(375);
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
        arm.up_arm_to_pos(350);
    }


}
