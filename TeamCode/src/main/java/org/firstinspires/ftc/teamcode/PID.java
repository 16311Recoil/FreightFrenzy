package org.firstinspires.ftc.teamcode;

public class PID {

    private double k_i;
    private double k_p;
    private double k_d;
    private double previousError;
    private double previousTime;
    private double target;
    private double i_sum = 0;
    //private final double MAX_SUM = 6000;


    public PID() {
        k_i = 0.0;
        k_p = 0.0;
        k_d = 0.0;

        target = 0.0;
        i_sum = 0.0;
        previousTime = 0;
        previousError = 0;

    }

    public PID(double k_p, double k_i, double k_d, double target) {
        this.k_i = k_i;
        this.k_p = k_p;
        this.k_d = k_d;

        this.target = target;
    }

    public void setConstants (double k_p, double k_i, double k_d, double target) {
        this.k_p = k_p;
        this.k_i = k_i;
        this.k_d = k_d;

        this.target = target;
    }

    public void reset(){
        target = 0.0;
        i_sum = 0.0;
        previousTime = 0;
        previousError = 0;
    }

    public double loop(double pos, double curTime) {
        double p, i, d;
        double error = target - pos;
        double deltaTime = curTime - previousTime;

        i_sum += 0.5 * (previousError + error) * deltaTime;
        /*
        if (i_sum > MAX_SUM){
            i_sum = MAX_SUM;
        }

         */

        p = error * k_p;
        i = i_sum * k_i;
        d = ((error - previousError) / deltaTime) * k_d;

        previousError = error;
        previousTime = curTime;
        return p + i + d;
    }

    public double getK_i() {
        return k_i;
    }

    public void setK_i(double k_i) {
        this.k_i = k_i;
    }

    public double getK_p() {
        return k_p;
    }

    public void setK_p(double k_p) {
        this.k_p = k_p;
    }

    public double getK_d() {
        return k_d;
    }

    public void setK_d(double k_d) {
        this.k_d = k_d;
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double getI_sum() {
        return i_sum;
    }
    public String toString(){
        return "Kp" + k_p + "\n" + "Ki" + k_i + "\n" + k_d + "kd" + "\n" + "Target" + target + "\n" + "PrevError" + previousError + "\n";
    }

}
