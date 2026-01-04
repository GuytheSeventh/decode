
package org.firstinspires.ftc.teamcode.opmode.auton;

import static org.firstinspires.ftc.teamcode.opmode.auton.GameConstants.*;

import org.firstinspires.ftc.teamcode.opmode.auton.Constant;

import com.acmerobotics.dashboard.config.Config;
// Red Constants
@Config
public class autoConstants {
    public static Constant START = new Constant(
    -24,-72,0);
    public static Constant farTip = new Constant(
            0,
            -60,
            -30);
    public static Constant closeTip = new Constant(
            0,
            0,
            -45);

    public static Constant back2wall = START;
    public static Constant closeBall = new Constant(
            -35,
            -33.5,
            90);
    public static Constant midBall = new Constant(
            -35,
            -9.5,
            90);
    public static Constant farBall = new Constant(
            -35,
            14.5,
            90);
    public static Constant moveABit = new Constant(
            -51,
            -57,
            40);
    public static Constant BASKET_7 = new Constant(
            -50,
            -56,
            40);
    public static Constant BASKET_8 = new Constant(
            -50,
            -56,
            40);
    public static Constant BASKET_9 = new Constant(
            -50,
            -56.0,
            40);
    public static Constant FAR_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 28,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2,
            UP + 1);
    public static Constant FAR_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 27,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 4.25,
            UP + 12.5);
    public static Constant CENTER_SAMPLE = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 14,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 - 3,
            UP + 2.5);
    public static Constant CENTER_SAMPLE_INT = new Constant(
            -TILE_LENGTH * 2.5 - BOT_WIDTH + 15,
            -TILE_LENGTH * 5 / 2 + BOT_LENGTH / 2 + 4.75,
            UP + 13.5);
    public static Constant WALL_SAMPLE = new Constant(
            -45,
            -33,
            160);
    public static Constant WALL_SAMPLE_INT = new Constant(
            -48,
            -31.5,
            160);
    public static Constant SUBMERSIBLE_1 = new Constant(
            -TILE_LENGTH - 2,
            -TILE_LENGTH / 2 + 3,
            0);
    public static Constant SUBMERSIBLE_2 = new Constant(
            -25,
            -TILE_LENGTH / 2 + 6,
            0);
    public static Constant SUBMERSIBLE_3 = new Constant(
            -25,
            -TILE_LENGTH / 2 + 6,
            0);
    public static Constant SUBMERSIBLE_4 = new Constant(
            -25,
            -TILE_LENGTH / 2 + 6,
            0);
    public static Constant SUBMERSIBLE_5 = new Constant(
            -25,
            -TILE_LENGTH / 2 + 6,
            0);
    public static double WALL_SAMPLE_DELAY = 1;
    public static double BASKET_WALL_DELAY = 0.7;
}
