package actions;

import common.Log;
import controller.action.ActionBoard;
import controller.ui.ui.components.DropInPointCounter;
import controller.ui.ui.components.Refreshable;
import data.states.AdvancedData;
import data.states.GamePreparationData;
import data.values.Side;
import org.junit.BeforeClass;
import org.junit.Test;

import static junit.framework.TestCase.assertEquals;

/**
 * Created by rkessler on 2017-07-27.
 */
public class TestDropinModeTransfersPoints {

    @BeforeClass
    public static void setUp(){
        System.setProperty("CONFIG_ROOT", "test_resources/");
        try {
            Log.init("dummy.log");
        } catch (Exception e){
            // TODO - make sure the logger is a proper singleton
        }
        ActionBoard.init(); // TODO - get rid of this stupid static thing
    }

    @Test
    public void testDropinPointSwitch(){
        AdvancedData data = new AdvancedData();

        DropInPointCounter dropInPointCounter = new DropInPointCounter();

        dropInPointCounter.initializeSide(Side.LEFT, 5, new Refreshable() {
            @Override
            public void refresh() {
                System.out.println("Refresh left");
            }
        });

        dropInPointCounter.initializeSide(Side.RIGHT, 5, new Refreshable() {
            @Override
            public void refresh() {
                System.out.println("Refresh right");
            }
        });

        dropInPointCounter.scoreGoal(data, Side.LEFT);

        assertEquals(1, dropInPointCounter.getPoints(Side.LEFT, 0));
        assertEquals(-1, dropInPointCounter.getPoints(Side.RIGHT, 0));

        data.sides_as_started = false;
        dropInPointCounter.update(data);

        assertEquals(-1, dropInPointCounter.getPoints(Side.LEFT, 0));
        assertEquals(1, dropInPointCounter.getPoints(Side.RIGHT, 0));

        dropInPointCounter.scoreGoal(data, Side.LEFT);

        assertEquals(0, dropInPointCounter.getPoints(Side.LEFT, 0));
        assertEquals(0, dropInPointCounter.getPoints(Side.RIGHT, 0));
    }
}
