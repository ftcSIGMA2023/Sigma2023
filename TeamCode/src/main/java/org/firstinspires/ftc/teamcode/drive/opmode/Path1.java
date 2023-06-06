package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.samples.SensorREV2mDistance;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@Autonomous(group = "drive", name = "Path1")
public class Path1 extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        double x, y = 0;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        Trajectory Mpole = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(40,20, Math.toRadians(90)), Math.toRadians(90)) //13.05V
                .build();

        Trajectory Hpole = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(42,0)) //13.05V
                .build();

        Trajectory Hpole2 = drive.trajectoryBuilder(Hpole.end())
                .lineToLinearHeading(new Pose2d(68,0,Math.toRadians(87)))
                .build();


        waitForStart();

        if (isStopRequested()) return;
        /*while((d1.getDistance(DistanceUnit.CM)<=5)||(d2.getDistance(DistanceUnit.CM)<=5)){
            if((d1.getDistance(DistanceUnit.CM)<=5)){
            Up_tt.setPosition(n);
            n+=0.05*(5-(d1.getDistance(DistanceUnit.CM)));}
            else if((d2.getDistance(DistanceUnit.CM)<=5)){
                Up_tt.setPosition(n);
                n+=-0.05*(5-(d1.getDistance(DistanceUnit.CM)));}
        }*/

        drive.followTrajectory(Mpole);
        //drive.followTrajectory(Hpole2);
        //drive.turn(Math.toRadians(90));

}}