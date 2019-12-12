package org.firstinspires.ftc.teamcode_12110titans;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Movement {

    double lF_power;
    double rF_power;
    double lB_power;
    double rB_power;
    double arm_power;

    public Movement(double x, double y, double turn, double power, boolean pullIn, boolean pullOut){

        double direction_travel;
        double direction_wheels;

        direction_travel=Math.atan2(y,x);
        direction_wheels=direction_travel-Math.PI/4;

        lF_power=Math.cos(direction_wheels)-turn;
        rF_power=Math.sin(direction_wheels)+turn;
        lB_power=Math.sin(direction_wheels)-turn;
        rB_power=Math.cos(direction_wheels)+turn;

        //A problem is that when you add power for direction and power for rotating, you can get power >1
        //So we had to adjust proportions
        if(Math.abs(lF_power)>1.0) {
            lF_power = lF_power / Math.abs(lF_power);
            rF_power = rF_power / Math.abs(lF_power);
            lB_power = lB_power / Math.abs(lF_power);
            rB_power = rB_power / Math.abs(lF_power);
        } else if (Math.abs(rF_power)>1.0) {
            lF_power = lF_power / Math.abs(rF_power);
            rF_power = rF_power / Math.abs(rF_power);
            lB_power = lB_power / Math.abs(rF_power);
            rB_power = rB_power / Math.abs(rF_power);
        } else if (Math.abs(lB_power)>1.0) {
            lF_power = lF_power / Math.abs(lB_power);
            rF_power = rF_power / Math.abs(lB_power);
            lB_power = lB_power / Math.abs(lB_power);
            rB_power = rB_power / Math.abs(lB_power);
        } else if (Math.abs(rB_power)>1.0) {
            lF_power = lF_power / Math.abs(rB_power);
            rF_power = rF_power / Math.abs(rB_power);
            lB_power = lB_power / Math.abs(rB_power);
            rB_power = rB_power / Math.abs(rB_power);
        } else {
            // do nothing
        }

        //Sometimes when your going straight (some exceptions0 the power to all the wheels is less then 1.

        //Also appreciate my wonderful code i made :)

        if(Math.abs(lF_power) < 1.0 && Math.abs(rF_power) < 1.0 && Math.abs(lB_power) < 1.0 && Math.abs(rB_power) < 1.0) {

            List<Double> powers = new ArrayList<Double>();

            powers.add(Math.abs(lF_power));
            powers.add(Math.abs(rF_power));
            powers.add(Math.abs(lB_power));
            powers.add(Math.abs(rB_power));

            double largest = Collections.max(powers);

            lF_power = lF_power / largest;
            rF_power = rF_power / largest;
            lB_power = lB_power / largest;
            rB_power = rB_power / largest;

        }





        if(pullIn==true){
            arm_power=1.0;
        }else if (pullOut==true){
            arm_power=-1.0;
        }else{
            arm_power=0.0;
        }

        this.arm_power=arm_power;
        this.lB_power=lB_power;
        this.lF_power=lF_power;
        this.rB_power=rB_power;
        this.rF_power=rF_power;

    }
}
/** © All Rights Reserved */