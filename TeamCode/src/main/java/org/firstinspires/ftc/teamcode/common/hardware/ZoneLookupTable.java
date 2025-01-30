package org.firstinspires.ftc.teamcode.common.hardware;

import java.util.HashMap;
import java.util.Map;

public class ZoneLookupTable {
    private final Map<Integer, double[]> zoneTable;

    public ZoneLookupTable() {
        zoneTable = new HashMap<>();
        // Populate with example values
        zoneTable.put(1, new double[]{3, 0.8});
        zoneTable.put(2, new double[]{7, 0.8});
        zoneTable.put(3, new double[]{11, 0.8});
        zoneTable.put(4, new double[]{3, 0.5});
        zoneTable.put(5, new double[]{7, 0.5});
        zoneTable.put(6, new double[]{11, 0.5});
    }

    public double[] getZoneInfo(int zone) {
        return zoneTable.get(zone);
    }

}
