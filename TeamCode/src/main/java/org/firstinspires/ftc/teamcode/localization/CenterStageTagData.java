package org.firstinspires.ftc.teamcode.localization;

import java.util.HashMap;
import java.util.Map;

public enum CenterStageTagData {

    BLUE_BACKDROP_LEFT(1, CenterStageFieldConstants.BACKDROP_X_OFFSET, 42, CenterStageFieldConstants.BACKDROP_TAG_HEIGHT, 90, 2),
    BLUE_BACKDROP_CENTER(2, CenterStageFieldConstants.BACKDROP_X_OFFSET, 48, CenterStageFieldConstants.BACKDROP_TAG_HEIGHT, 90, 2),
    BLUE_BACKDROP_RIGHT(3, CenterStageFieldConstants.BACKDROP_X_OFFSET, 54, CenterStageFieldConstants.BACKDROP_TAG_HEIGHT, 90, 2),
    RED_BACKDROP_LEFT(4, CenterStageFieldConstants.BACKDROP_X_OFFSET, -42, CenterStageFieldConstants.BACKDROP_TAG_HEIGHT, 90, 2),
    RED_BACKDROP_CENTER(5, CenterStageFieldConstants.BACKDROP_X_OFFSET, -48, CenterStageFieldConstants.BACKDROP_TAG_HEIGHT, 90, 2),
    RED_BACKDROP_RIGHT(6, CenterStageFieldConstants.BACKDROP_X_OFFSET, -54, CenterStageFieldConstants.BACKDROP_TAG_HEIGHT, 90, 2),
    RED_AUDIENCE_WALL_BIG(7, CenterStageFieldConstants.AUDIENCE_WALL_X_OFFSET, -39.5, CenterStageFieldConstants.AUDIENCE_WALL_TAG_HEIGHT, 270, 5),
    RED_AUDIENCE_WALL_SMALL(8, CenterStageFieldConstants.AUDIENCE_WALL_X_OFFSET, -39.5, CenterStageFieldConstants.AUDIENCE_WALL_TAG_HEIGHT, 270, 2),
    BLUE_AUDIENCE_WALL_BIG(9, CenterStageFieldConstants.AUDIENCE_WALL_X_OFFSET, 39.5, CenterStageFieldConstants.AUDIENCE_WALL_TAG_HEIGHT, 270, 5),
    BLUE_AUDIENCE_WALL_SMALL(10, CenterStageFieldConstants.AUDIENCE_WALL_X_OFFSET, 39.5, CenterStageFieldConstants.AUDIENCE_WALL_TAG_HEIGHT, 270, 2);

    public final int id;
    public final double x;
    public final double y;
    public final double z;
    public final double heading;
    public final double size;

    CenterStageTagData(int id, double x, double y, double z, double heading, double size) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.z = z;
        this.heading = heading;
        this.size = size;
    }

    public static Map<Integer, CenterStageTagData> getTagMap() {
        CenterStageTagData[] tagData = CenterStageTagData.values();

        Map<Integer, CenterStageTagData> tagMap = new HashMap<>();
        for (CenterStageTagData tag : tagData) {
            tagMap.put(tag.id, tag);
        }
        return tagMap;
    }

}
