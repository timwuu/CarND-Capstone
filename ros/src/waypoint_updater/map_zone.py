#!/usr/bin/env python

class MapZone(object):
    def __init__(self):
        self.map_zone={}

    def clear(self):
        self.map_zone={}

    def getKey(self, x, y):
        return int(round(x/10.0)*100000+round(y/10.0)*10)

    def addElement( self, i, x, y):
        key = self.getKey( x, y)
        try:
            self.map_zone[key].append(i)
        except:
            self.map_zone[key]=[]
            self.map_zone[key].append(i)

    def zoning(self, map_x, map_y):
        k = len(map_x)

        for i in range(0,k):
            key = self.getKey( map_x[i], map_y[i])

            try:
                self.map_zone[key].append(i)
            except:
                self.map_zone[key]=[]
                self.map_zone[key].append(i)

    def getZoneElements(self, x, y):
        elems = None
        try:
            elems = self.map_zone[ self.getKey(x,y)]
        except:
            return None
        return elems

    def getZoneNeighborElements(self, x, y):

        elems = []

        for s in [ x-10.0, x, x+10.0]:
            for t in [ y-10.0, y, y+10.0]:
                try:
                    elems = elems + self.map_zone[ self.getKey(s,t)]
                except:
                    continue

        return elems

# mz = MapZone()

# map_x=[ 5100.2, 200.2, 4000.30,  2321.06, 3240.03, 5101.0, 5109.1]
# map_y=[ 2100.2, 3200.2, 1500.30,  21.06, 240.03, 2099.0, 2100.0]

# mz.zoning( map_x, map_y)

# print mz.map_zone

# print mz.getZoneNeighborElements( 5100, 2100)

