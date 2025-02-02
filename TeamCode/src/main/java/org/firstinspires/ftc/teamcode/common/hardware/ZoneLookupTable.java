package org.firstinspires.ftc.teamcode.common.hardware;

import java.util.HashMap;
import java.util.Map;

public class ZoneLookupTable {
    private final Map<Integer, double[]> zoneTable;

    public ZoneLookupTable() {
        zoneTable = new HashMap<>();
        // Populate with example values
        zoneTable.put(1, new double[]{-1, 0.65});
        zoneTable.put(2, new double[]{7, 0.65});
        zoneTable.put(3, new double[]{-1, 0.45});
        zoneTable.put(4, new double[]{7, 0.45});
    }

    public double[] getZoneInfo(int zone) {
        return zoneTable.get(zone);
    }

}
