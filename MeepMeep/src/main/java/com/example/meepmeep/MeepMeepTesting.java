package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(60, 50, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-37.87, 11.58, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-49.71, 20.11), Math.toRadians(90.00))
                .splineTo(new Vector2d(-50, 61.90), Math.toRadians(90.00))
                .setReversed(true)
                .splineTo(new Vector2d(-50, 17.85), Math.toRadians(90.00))

                .build());

        Image fieldBackground = null;
        try { fieldBackground = ImageIO.read(new File("/Users/rrrr/Juice-INTO-THE-DEEP-Black.png")); }
        catch(IOException ignored) {}

        meepMeep.setBackground(fieldBackground)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}