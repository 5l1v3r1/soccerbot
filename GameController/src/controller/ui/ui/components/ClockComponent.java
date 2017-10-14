package controller.ui.ui.components;

import common.TotalScaleLayout;
import controller.action.ActionBoard;
import controller.ui.helper.FontHelper;
import controller.ui.ui.customized.ImageButton;
import controller.ui.ui.customized.ImagePanel;
import data.Helper;
import data.Rules;
import data.states.AdvancedData;
import data.values.GameStates;

import javax.swing.*;
import java.awt.*;
import java.util.concurrent.ConcurrentLinkedDeque;


/**
 * Created by rkessler on 2017-03-28.
 */
public class ClockComponent extends AbstractComponent {

    private JLabel clock;
    private JLabel clockSub;
    private ImageButton incGameClock;
    private ImageButton clockPause;
    private ImageButton clockReset;


    private static final String CLOCK_RESET = "reset.png";
    private static final String CLOCK_PAUSE = "pause.png";
    private static final String CLOCK_PLAY = "play.png";
    private static final String CLOCK_PLUS = "plus.png";

    private static final String BACKGROUND_CLOCK_SMALL = "time_ground_small.png";
    private static final String BACKGROUND_CLOCK = "time_ground.png";

    private static final String ICONS_PATH = "config/icons/";

    private static final int KICKOFF_BLOCKED_HIGHLIGHT_SECONDS = 3;

    private static final Color COLOR_HIGHLIGHT = Color.YELLOW;
    private static final Color SUB_CLOCK_COLOUR = Color.LIGHT_GRAY;
    private static final Color CLOCK_COLOUR = Color.WHITE;

    private ImageIcon clockImgReset;
    private ImageIcon clockImgPlay;
    private ImageIcon clockImgPause;
    private ImageIcon clockImgPlus;

    public ClockComponent() {
        clockImgReset = new ImageIcon(ICONS_PATH + CLOCK_RESET);
        clockImgPlay = new ImageIcon(ICONS_PATH + CLOCK_PLAY);
        clockImgPlus = new ImageIcon(ICONS_PATH + CLOCK_PLUS);
        clockImgPause = new ImageIcon(ICONS_PATH + CLOCK_PAUSE);

        defineLayout();
    }

    public void defineLayout() {
        //--mid--
        //  time
        clockReset = new ImageButton(clockImgReset.getImage());
        clockReset.setOpaque(false);
        clockReset.setBorder(null);
        ImagePanel clockContainer;
        if (Rules.league.lostTime) {
            clockContainer = new ImagePanel(new ImageIcon(ICONS_PATH + BACKGROUND_CLOCK_SMALL).getImage());
        } else {
            clockContainer = new ImagePanel(new ImageIcon(ICONS_PATH + BACKGROUND_CLOCK).getImage());
        }
        clockContainer.setOpaque(false);

        clock = new JLabel("10:00");
        clock.setForeground(CLOCK_COLOUR);
        clock.setHorizontalAlignment(JLabel.CENTER);

        clockPause = new ImageButton(clockImgReset.getImage());
        clockPause.setOpaque(false);
        clockPause.setBorder(null);

        clockSub = new JLabel("0:00");
        clockSub.setHorizontalAlignment(JLabel.CENTER);
        clockSub.setForeground(SUB_CLOCK_COLOUR);

        incGameClock = new ImageButton(clockImgPlus.getImage());
        incGameClock.setOpaque(false);
        incGameClock.setBorder(null);

        clock.setVisible(true);
        clockPause.setVisible(true);
        clockSub.setVisible(true);


        TotalScaleLayout tsl = new TotalScaleLayout(this);

        tsl.add(0, 0, 0.85, 0.5, clock);
        tsl.add(0, 0.5, 0.85, 0.5, clockSub);

        tsl.add(0.05, 0.05, 0.75, 0.9, clockContainer);

        tsl.add(0.85, 0, 0.15, 0.33, clockPause);
        tsl.add(0.85, 0.33, 0.15, 0.33, clockReset);
        tsl.add(0.85, 0.66, 0.15, 0.34, incGameClock);

        clockReset.addActionListener(ActionBoard.clockReset);
        clockPause.addActionListener(ActionBoard.clockPause);
        if (Rules.league.lostTime) {
            incGameClock.addActionListener(ActionBoard.incGameClock);
        } else {
            incGameClock.setVisible(false);
        }

        this.setLayout(tsl);
        this.setVisible(true);
    }




    @SuppressWarnings("Duplicates")
    @Override
    public void update(AdvancedData data) {
        // Set the font
        clock.setFont(FontHelper.timeFont);
        clockSub.setFont(FontHelper.timeSubFont);

        clock.setText(Helper.formatTime(data.getRemainingGameTime(true)));

        Integer secondaryTime = data.getSecondaryTime(KICKOFF_BLOCKED_HIGHLIGHT_SECONDS - 1);
        if (secondaryTime != null) {
            if (data.gameState == GameStates.PLAYING) {
                clockSub.setText(Helper.formatTime(Math.max(0, secondaryTime)));
                clockSub.setForeground(secondaryTime <= 0
                        && clockSub.getForeground() != COLOR_HIGHLIGHT ? COLOR_HIGHLIGHT : SUB_CLOCK_COLOUR);
            } else {
                clockSub.setText(Helper.formatTime(secondaryTime));
                clockSub.setForeground(SUB_CLOCK_COLOUR);
            }
        } else {
            clockSub.setText("");
            clockSub.setForeground(SUB_CLOCK_COLOUR);
        }

        ImageIcon tmp;
        if (ActionBoard.clock.isClockRunning(data)) {
            tmp = clockImgPause;
        } else {
            tmp = clockImgPlay;
        }
        clockPause.setImage(tmp.getImage());
        clockReset.setVisible(ActionBoard.clockReset.isLegal(data));
        clockPause.setVisible(ActionBoard.clockPause.isLegal(data));
        if (Rules.league.lostTime) {
            incGameClock.setEnabled(ActionBoard.incGameClock.isLegal(data));
        }
    }
}
