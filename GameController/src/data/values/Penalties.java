package data.values;

import controller.ui.localization.LocalizationManager;

/**
 * Created by rkessler on 2017-03-25.
 */
public enum Penalties implements DocumentingMarkdown {

    UNKNOWN(255, "Unknown", -1),

    NONE(0, "None", -1),

    SPL_ILLEGAL_BALL_CONTACT(1, LocalizationManager.getLocalization().ILLEGAL_BALL_CONTACT, 45),
    SPL_PLAYER_PUSHING(2, LocalizationManager.getLocalization().PUSHING, 45),
    SPL_ILLEGAL_MOTION_IN_SET(3, "illegal motion in set", 0),
    SPL_INACTIVE_PLAYER(4, "inactive", 45),
    SPL_ILLEGAL_DEFENDER(5, "illegal defender", 45),
    SPL_LEAVING_THE_FIELD(6, "leaving the field", 45),
    SPL_KICK_OFF_GOAL(7, "kickoff goal", 45),
    SPL_REQUEST_FOR_PICKUP(8, "request for pickup", 45),
    SPL_COACH_MOTION(9, "coach motion", 2*10*60),

    SUBSTITUTE(14, "substitute", -1), // TODO check if different for SPL than HL and what value is
    MANUAL(15, "manual", -1), // TODO check if different for SPL than HL and what value is

    HL_BALL_MANIPULATION(30, "ball_manipulation", 30),
    HL_PHYSICAL_CONTACT(31, "pushing", 30),
    HL_ILLEGAL_ATTACK(32, "illegal attack", 30),
    HL_ILLEGAL_DEFENSE(33, "illegal defender", 30),
    HL_PICKUP_OR_INCAPABLE(34, "pickup/incapable", 30),
    HL_SERVICE(35, "service", 60);

    private byte byte_value;
    private String humanReadable;
    private int penaltyTime;

    Penalties(int byte_value, String humanReadable, int penalty_time) {
        this.byte_value = (byte) byte_value;
        this.humanReadable = humanReadable;
        this.penaltyTime = penalty_time;
    }

    public byte value() {
        return byte_value;
    }

    public int penaltyTime(){
        return this.penaltyTime;
    }
    public static Penalties fromValue(byte b) {
        for(Penalties penalty : Penalties.values()){
            if (penalty.value() == b){
                return penalty;
            }
        }
        System.out.println("Warning: Could not resolve Penalty byte.");
        return UNKNOWN;
    }

    public String toString(){
        return this.humanReadable;
    }
}
