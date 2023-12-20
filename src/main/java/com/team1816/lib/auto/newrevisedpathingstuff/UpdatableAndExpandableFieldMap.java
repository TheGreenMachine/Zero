package com.team1816.lib.auto.newrevisedpathingstuff;

public class UpdatableAndExpandableFieldMap {
    private final FieldMap stableMap;
    private final FieldMap updatableMap;
    private FieldMap currentMap;

    private boolean updatableMapChanged;

    private int mapX;
    private int mapY;
    private double expansionWidth;

    /**
     * @param mapX
     * @param mapY
     * @param stableMap a FieldMapTest of all nonupdating obstacles of the field
     * @param updatableMap a FieldMapTest of all updating obstacles of the field
     */
    public UpdatableAndExpandableFieldMap(int mapX, int mapY, FieldMap stableMap, FieldMap updatableMap, double expansionWidth){
        assert(stableMap.mapSizeEqual(updatableMap));
        assert(stableMap.getMapX() == mapX && stableMap.getMapY() == mapY);

        this.stableMap = stableMap.getCopy();
        stableMap = getExpandedMap(stableMap, expansionWidth);

        this.updatableMap = updatableMap.getCopy();

        currentMap = new FieldMap(mapX, mapY);
        currentMap.addOtherMap(stableMap);
        currentMap.addOtherMap(getExpandedMap(updatableMap, expansionWidth));

        updatableMapChanged = false;

        this.mapX = mapX;
        this.mapY = mapY;
        this.expansionWidth = expansionWidth;
    }

    private FieldMap getExpandedMap(FieldMap map, double expansionWidth){
        FieldMap expandedMap = map.getCopy();

        if(expansionWidth <= 0)
            return expandedMap;

        for(int j = 0; j < map.getMapY(); j++){
            for(int i = 0; i < map.getMapX(); i++){
                if(map.checkPixelHasObject(i, j)) {
                    expandedMap.drawPixel(i+1, j);
                    expandedMap.drawPixel(i-1, j);
                    expandedMap.drawPixel(i, j+1);
                    expandedMap.drawPixel(i, j-1);

                    boolean north = map.checkPixelHasObjectOrOffMap(i, j + 1);
                    boolean east = map.checkPixelHasObjectOrOffMap(i + 1, j);
                    boolean south = map.checkPixelHasObjectOrOffMap(i, j - 1);
                    boolean west = map.checkPixelHasObjectOrOffMap(i - 1, j);

                    int xShift = 0;
                    int yShift = 0;
                    int numNeighbors = 0;

                    if (north) {
                        numNeighbors++;
                        yShift--;
                    }
                    if (east) {
                        numNeighbors++;
                        xShift--;
                    }
                    if (south) {
                        numNeighbors++;
                        yShift++;
                    }
                    if (west) {
                        numNeighbors++;
                        xShift++;
                    }

                    expandedMap.drawPixel(i, j);

                    double newExpansionWidth = expansionWidth;

                    //Don't ask me why it's this number, for some reason it makes the program work
                    if(expansionWidth <= 1.57)
                        newExpansionWidth = 1;
                    else if(expansionWidth <= 2)
                        newExpansionWidth = 2;

                    if (numNeighbors == 4) {
                        continue;
                    } else if (numNeighbors == 3) {
                        for (int stepCoefficient = 1; stepCoefficient <= newExpansionWidth; stepCoefficient++)
                            expandedMap.drawPixel(i + (xShift * stepCoefficient), j + (yShift * stepCoefficient));
                    } else if (numNeighbors == 0) {
                        expandedMap.drawCircle(i, j, expansionWidth, true);
                    } else if (numNeighbors == 2 && north && south) {
                        for (int stepCoefficient = 1; stepCoefficient <= newExpansionWidth; stepCoefficient++) {
                            expandedMap.drawPixel(i + stepCoefficient, j);
                            expandedMap.drawPixel(i - stepCoefficient, j);
                        }
                    } else if (numNeighbors == 2 && east && west) {
                        for (int stepCoefficient = 1; stepCoefficient <= newExpansionWidth; stepCoefficient++) {
                            expandedMap.drawPixel(i, j + stepCoefficient);
                            expandedMap.drawPixel(i, j - stepCoefficient);
                        }
                    } else {
                        boolean quadrant1 = !north && !east;
                        boolean quadrant2 = !west && !north;
                        boolean quadrant3 = !south && !west;
                        boolean quadrant4 = !east && !south;

                        expandedMap.drawCircleQuadrant(i, j, quadrant1, quadrant2, quadrant3, quadrant4, expansionWidth, true);
                    }
                }
            }
        }

        return expandedMap;
    }

    public boolean isPerfectOverlay(){
        OverlayedFieldMaps overlayedMaps = new OverlayedFieldMaps(mapX, mapY);

        overlayedMaps.addFieldMap(this.getTargetExpandedMap(stableMap));
        overlayedMaps.addFieldMap(currentMap);

        return overlayedMaps.isPerfectOverlay();
    }

    public void updateCurrentMap(){
        if(updatableMapChanged) {
            currentMap = stableMap.getCopy();
            currentMap.addOtherMap(getExpandedMap(updatableMap, expansionWidth));
        }
    }

    public String printExpansionMapTest(){
        OverlayedFieldMaps overlayedMaps = new OverlayedFieldMaps(mapX, mapY);

        overlayedMaps.addFieldMap(this.getTargetExpandedMap(stableMap));
        overlayedMaps.addFieldMap(currentMap);

        return overlayedMaps.toStringIndicateOverlays();
    }

    public String printOverlayedMaps(boolean indicateOverlay, boolean overlayStableMap, boolean overlayUpdatableMap, boolean overlayCurrentMap){
        OverlayedFieldMaps overlayedMaps = new OverlayedFieldMaps(mapX, mapY);

        if(overlayStableMap)
            overlayedMaps.addFieldMap(stableMap);
        if(overlayUpdatableMap)
            overlayedMaps.addFieldMap(updatableMap);
        if(overlayCurrentMap)
            overlayedMaps.addFieldMap(currentMap);

        if(indicateOverlay)
            return overlayedMaps.toStringIndicateOverlays();
        else
            return overlayedMaps.toString();
    }

    public FieldMap getTargetExpandedMap(FieldMap map){
        FieldMap targetExpandedMap = new FieldMap(map.getMapX(), map.getMapY());

        for(int j = 0; j < map.getMapY(); j++)
            for(int i = 0; i < map.getMapX(); i++)
                if(map.checkPixelHasObject(i, j))
                    targetExpandedMap.drawCircle(i, j, expansionWidth, true);

        return targetExpandedMap;
    }

    public int[][] getOverlayedMaps(boolean overlayStableMap, boolean overlayUpdatableMap, boolean overlayCurrentMap){
        OverlayedFieldMaps overlayedMaps = new OverlayedFieldMaps(mapX, mapY);

        if(overlayStableMap)
            overlayedMaps.addFieldMap(stableMap);
        if(overlayUpdatableMap)
            overlayedMaps.addFieldMap(updatableMap);
        if(overlayCurrentMap)
            overlayedMaps.addFieldMap(currentMap);

        return overlayedMaps.getOverlayedFieldMaps();
    }

    public FieldMap getCurrentMap() {
        return currentMap;
    }

    public FieldMap getUpdatableMap(){
        updatableMapChanged = true;

        return updatableMap;
    }
}