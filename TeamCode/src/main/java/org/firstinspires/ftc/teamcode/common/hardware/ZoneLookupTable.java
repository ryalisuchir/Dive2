package org.firstinspires.ftc.teamcode.common.hardware;

import java.util.HashMap;
import java.util.Map;

public class ZoneLookupTable {
    private final Map<Integer, double[]> zoneTable;

    public ZoneLookupTable() {
        zoneTable = new HashMap<>();
        zoneTable.put(1, new double[]{0, 0.7}); //zone 1 (top left)
        zoneTable.put(2, new double[]{0, 0.7}); //zone 2 (top right)
        zoneTable.put(3, new double[]{0, 0.45}); //zone 3 (bottom left)
        zoneTable.put(4, new double[]{0, 0.45}); //zone 4 (bottom right)
    }

    public double[] getZoneInfo(int zone) {
        return zoneTable.get(zone);
    }

}
