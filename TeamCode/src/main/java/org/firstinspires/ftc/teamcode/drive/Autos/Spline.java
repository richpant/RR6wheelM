package org.firstinspires.ftc.teamcode.drive.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Spline")
public class Spline extends LinearOpMode {

    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .60, correction;// this is where you change power for all IMU moves


    @Override
    public void runOpMode() {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        //lift.getZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(15)//forward
                .turn(90)//turn
                .forward(15)//forward
                .turn(90)//turn
                .forward(15)//forward
                .turn(90)
                .forward(15)//should make square
                .lineToLinearHeading(new Pose2d(40, 40, Math.toRadians(90)))//linear move with rotation
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))//return to start
                .splineTo(new Vector2d(40, 40), Math.toRadians(0))
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build();
        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);


    }
}
