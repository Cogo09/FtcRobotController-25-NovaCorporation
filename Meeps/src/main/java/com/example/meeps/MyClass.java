package com.example.meeps;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Image img = null;
        try {
            // 2. Load your custom BIOBUZZ PNG file

            img = ImageIO.read(new File("C:\\Users\\coeng\\Documents\\GitHub\\FtcRobotController-25-NovaCorp\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\biobuzzfield.png"));
        } catch (IOException e) {
            System.out.println("Error: Could not find or load the field image!");
            e.printStackTrace();
        }

        // Your robot configuration and trajectory building goes here
        // ...

        meepMeep.setBackground(img) // 3. Pass your image object here
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                // Add your robot instance and start
                .start();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(120, 120, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-56, 60, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(0)), Math.toRadians(270))
                //First shot here
                .lineToY(20)
                .splineToLinearHeading(new Pose2d(-26, 45, Math.toRadians(90)), Math.toRadians(270))
                .lineToY(55)
                //collection here
                .lineToY(50)
                .splineToLinearHeading(new Pose2d(45, 40, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(50, 10, Math.toRadians(180)), Math.toRadians(180))
                .lineToX(60)
                //second shot here
                .lineToX(50)
                .splineToLinearHeading(new Pose2d(10, 45, Math.toRadians(90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-60, 40, Math.toRadians(90)), Math.toRadians(270))
                .lineToY(60)
                //collection here
                .lineToY(50)
                .splineToLinearHeading(new Pose2d(10, 45, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(55, 10, Math.toRadians(180)), Math.toRadians(180))
                .lineToX(60)
                //third shot here
                .lineToX(50)
                .splineToLinearHeading(new Pose2d(45, 60, Math.toRadians(270)), Math.toRadians(0))
                //park



//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
                .build());

        meepMeep.setBackground(img) // 3. Pass your image object here
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                // Add your robot instance and start
                .start();
    }
}