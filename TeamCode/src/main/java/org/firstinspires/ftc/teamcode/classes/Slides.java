package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

@Config
public class Slides {
    public DcMotorEx gl = null; 
    public DcMotorEx gr = null;

    private PIDController controllerl;
    private PIDController controllerr;

    public static double p = 0.01, i = 0.00015, d = 0.000135;
    public static double f = 0.001;

    public static double maxxPIDpower = 0.9;

    public static int target;

    double ticks_in_degrees = 384.5 / 360;

    public Slides(HardwareMap hardwareMap)
    {
        target = 0;

        gl = hardwareMap.get(DcMotorEx.class, "gl");
        gr = hardwareMap.get(DcMotorEx.class, "gr");

        gr.setDirection(DcMotorSimple.Direction.REVERSE);

        gl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gl.setPower(0);
        gr.setPower(0);

        controllerl = new PIDController(p, i, d);
        controllerr = new PIDController(p, i, d);
    }

    public void update()
    {
        if(target == 0 && gl.getCurrentPosition() < 50 && gr.getCurrentPosition() < 50)
        {
            gr.setPower(0);
            gl.setPower(0);
        }
        else
        {
            controllerl.setPID(p, i, d);
            int glPos = gl.getCurrentPosition();
            double pidl = Math.min(controllerl.calculate(glPos, target), maxxPIDpower);
            double ffl = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double powerl = pidl + ffl;

            gl.setPower(powerl);

            controllerr.setPID(p, i, d);
            int grPos = gr.getCurrentPosition();
            double pidr = Math.min(controllerr.calculate(grPos, target), maxxPIDpower);
            double ffr = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double powerr = pidr + ffr;

            gr.setPower(powerr);
        }
    }

    public void down()
    {
        target = 0;
    }
    public void up(int level)
    {
        if(level == 1)
            target = 600;
        else if(level == 2)
            target = 900;
        else if(level == 3)
            target = 1300;
        else if(level == 4){

            gl.setTargetPosition(gl.getCurrentPosition() - 237);
            gr.setTargetPosition(gr.getCurrentPosition() - 237);

            gl.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            gr.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            gl.setPower(0.8);
            gr.setPower(0.8);

            gl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            gr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
        else if(level == 5)
            target = 1650;
        else
            target = 0;
    }
}