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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(56, -60, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(62, -12, Math.toRadians(180)), Math.toRadians(0))
                //SHOT
                .lineToX(58)
                .splineToLinearHeading(new Pose2d(25, -52, Math.toRadians(270)), Math.toRadians(270))
                .lineToY(-60)
                //Collection
                .lineToY(-52)
                .splineToLinearHeading(new Pose2d(-45, -34, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-62, -10, Math.toRadians(0)), Math.toRadians(180))
                //SHOT
                .lineToX(58)
                .turnTo(Math.toRadians(270))
                .lineToY(-60)
                //collection
                        .lineToY(-55)
                .splineToLinearHeading(new Pose2d(-45, -34, Math.toRadians(0)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-62, -10, Math.toRadians(0)), Math.toRadians(180))
                //SHOT
                        .lineToX(-58)
                .splineToLinearHeading(new Pose2d(-34, -65, Math.toRadians(90)), Math.toRadians(270))
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