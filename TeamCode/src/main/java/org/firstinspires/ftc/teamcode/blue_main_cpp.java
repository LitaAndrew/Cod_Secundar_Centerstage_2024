package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.CupaCatinca;
import org.firstinspires.ftc.teamcode.classes.Slides;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="blue_main_cpp")
public class blue_main_cpp extends LinearOpMode {

    DcMotorEx intake;

    SampleMecanumDrive drive;
    Slides lift;
    CupaCatinca catincaMamica;
    ElapsedTime runtime = new ElapsedTime();
    //ElapsedTime start = new ElapsedTime();

  /*  final Thread update_it = new Thread() {
        public void run() {
            lift.up(2);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            catincaMamica.put();
            catincaMamica.adunaPixeli();
            try {
                sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            catincaMamica.returnHome();
            try {
                sleep(250);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            lift.up(0);
        }
    };
*/
    @Override
    public void runOpMode() throws InterruptedException {

    //    intake = hardwareMap.get(DcMotorEx.class, "intake");
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Slides(hardwareMap);
        catincaMamica = new CupaCatinca(hardwareMap);

        Pose2d startPose = new Pose2d(36, 60, Math.toRadians(90));

        Trajectory mergi = drive.trajectoryBuilder(new Pose2d())
                .back(21.9)
                .build();
        Trajectory mergi2 = drive.trajectoryBuilder(new Pose2d())
                .forward(4)
                .build();

        Trajectory mergi3 = drive.trajectoryBuilder(new Pose2d())
                .back(11.8)
                .build();

        Trajectory spate = drive.trajectoryBuilder(new Pose2d())
                .back(15.5)
                .build();

        catincaMamica.returnHome();
        waitForStart();
        drive.followTrajectory(mergi);
        drive.turn(Math.toRadians(83.5));
        drive.followTrajectory(spate);
    //    drive.turn(Math.toRadians(-88));
    //    drive.followTrajectory(mergi);
        runtime.reset();

        if (isStopRequested()) return;




        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 9) {
            if (runtime.seconds() > 0.8 && runtime.seconds() < 6) {
                lift.up(3);
            }
            if(runtime.seconds() > 1.8 && runtime.seconds() < 3){
                catincaMamica.put();
            }
            if(runtime.seconds() > 3 && runtime.seconds() < 6){
                catincaMamica.adunaPixeli();
            }
            if(runtime.seconds() > 5){
                catincaMamica.returnHome();
                catincaMamica.stopCatinca();
            }
            if(runtime.seconds() > 6)
                lift.up(0);

            telemetry.addData("merge de:``", runtime);
            telemetry.update();
            lift.update();
        }
            drive.followTrajectory(mergi2);
            drive.turn(Math.toRadians(-87));
            drive.followTrajectory(mergi);

    }
}




        //  while (opModeIsActive() && runtime.seconds() < 2) {
      //      if (runtime.seconds() > 0.1 && runtime.seconds() < 1.5) {
      //          lift.up(2);




       /* TrajectorySequence tr1 = drive.trajectorySequenceBuilder(new Pose2d(31.45, -63.28, Math.toRadians(90.00)))
                .addTemporalMarker(0,()->{

                })
                .splineTo(new Vector2d(37.89, -18.38), Math.toRadians(94.04))
                .splineToLinearHeading(new Pose2d(46.49, -15.08, Math.toRadians(-30.00)), Math.toRadians(23.79))
                .build();*/