package org.firstinspires.ftc.teamcode.util;

public class Mathematics {
    public static double clamp(double value, double min, double max){
        if(value < min){
            value = min;
        }

        if(value > max){
            value = max;
        }

        return value;
    }
}
