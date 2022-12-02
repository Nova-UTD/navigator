import math


class ShadowCaster:

    def __init__(self, mapwidth, mapheight):
        self.width = mapwidth
        self.height = mapheight
        self.visiblemap = []
        self.seethrough = []
        self.sourcex = 0
        self.sourcey = 0
        self.range = 0

    def cast_shadow(self, seethrough, sourcex, sourcey, sightrange):
        self.sourcex = sourcex
        self.sourcey = sourcey
        self.range = sightrange
        self.seethrough = seethrough
        self.visiblemap = [[False for y in range(self.height)]
                           for x in range(self.width)]
        self.visiblemap[sourcex][sourcey] = True
        for octant in range(1, 9):
            self._scan(1, octant, 1.0, 0.0)
        return self.visiblemap

    def _scan(self, depth, octant, startslope, endslope):
        x = 0
        y = 0
        if octant == 1:   # NW
            x = self.sourcex - int(startslope * depth)
            y = self.sourcey - depth
            if self._check_bounds(x, y):
                while self._get_slope(x, y, self.sourcex,
                                      self.sourcey) >= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x - 1, y, False):
                                startslope = self._get_slope(x - .5, y - .5,
                                                             self.sourcex,
                                                             self.sourcey)
                        else:
                            if self._test_tile(x - 1, y, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_slope(x - .5, y + .5,
                                                           self.sourcex,
                                                           self.sourcey))
                        self.visiblemap[x][y] = True
                    x += 1
                x -= 1
        elif octant == 2:   # NE
            x = self.sourcex + int(startslope * depth)
            y = self.sourcey - depth
            if self._check_bounds(x, y):
                while self._get_slope(x, y, self.sourcex,
                                      self.sourcey) <= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x + 1, y, False):
                                startslope = -self._get_slope(x + .5, y - .5,
                                                              self.sourcex,
                                                              self.sourcey)
                        else:
                            if self._test_tile(x + 1, y, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_slope(x + .5, y + .5,
                                                           self.sourcex,
                                                           self.sourcey))
                        self.visiblemap[x][y] = True
                    x -= 1
                x += 1
        elif octant == 3:   # EN
            x = self.sourcex + depth
            y = self.sourcey - int(startslope * depth)
            if self._check_bounds(x, y):
                while self._get_inv_slope(x, y, self.sourcex,
                                          self.sourcey) <= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x, y - 1, False):
                                startslope = -self._get_inv_slope(x + .5,
                                                                  y - .5,
                                                                  self.sourcex,
                                                                  self.sourcey)
                        else:
                            if self._test_tile(x, y - 1, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_inv_slope(x - .5, y - .5,
                                                               self.sourcex,
                                                               self.sourcey))
                        self.visiblemap[x][y] = True
                    y += 1
                y -= 1
        elif octant == 4:   # ES
            x = self.sourcex + depth
            y = self.sourcey + int(startslope * depth)
            if self._check_bounds(x, y):
                while self._get_inv_slope(x, y, self.sourcex,
                                          self.sourcey) >= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x, y + 1, False):
                                startslope = self._get_inv_slope(x + .5,
                                                                 y + .5,
                                                                 self.sourcex,
                                                                 self.sourcey)
                        else:
                            if self._test_tile(x, y + 1, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_inv_slope(x - .5, y + .5,
                                                               self.sourcex,
                                                               self.sourcey))
                        self.visiblemap[x][y] = True
                    y -= 1
                y += 1
        elif octant == 5:   # SE
            x = self.sourcex + int(startslope * depth)
            y = self.sourcey + depth
            if self._check_bounds(x, y):
                while self._get_slope(x, y, self.sourcex,
                                      self.sourcey) >= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x + 1, y, False):
                                startslope = self._get_slope(x + .5, y + .5,
                                                             self.sourcex,
                                                             self.sourcey)
                        else:
                            if self._test_tile(x + 1, y, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_slope(x + .5, y - .5,
                                                           self.sourcex,
                                                           self.sourcey))
                        self.visiblemap[x][y] = True
                    x -= 1
                x += 1
        elif octant == 6:   # SW
            x = self.sourcex - int(startslope * depth)
            y = self.sourcey + depth
            if self._check_bounds(x, y):
                while self._get_slope(x, y, self.sourcex,
                                      self.sourcey) <= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x - 1, y, False):
                                startslope = -self._get_slope(x - .5, y + .5,
                                                              self.sourcex,
                                                              self.sourcey)
                        else:
                            if self._test_tile(x - 1, y, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_slope(x - .5, y - .5,
                                                           self.sourcex,
                                                           self.sourcey))
                        self.visiblemap[x][y] = True
                    x += 1
                x -= 1
        elif octant == 7:   # WS
            x = self.sourcex - depth
            y = self.sourcey + int(startslope * depth)
            if self._check_bounds(x, y):
                while self._get_inv_slope(x, y, self.sourcex,
                                          self.sourcey) <= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x, y + 1, False):
                                startslope = -self._get_inv_slope(x - .5,
                                                                  y + .5,
                                                                  self.sourcex,
                                                                  self.sourcey)
                        else:
                            if self._test_tile(x, y + 1, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_inv_slope(x + .5, y + .5,
                                                               self.sourcex,
                                                               self.sourcey))
                        self.visiblemap[x][y] = True
                    y -= 1
                y += 1
        elif octant == 8:   # WN
            x = self.sourcex - depth
            y = self.sourcey - int(startslope * depth)
            if self._check_bounds(x, y):
                while self._get_inv_slope(x, y, self.sourcex,
                                          self.sourcey) >= endslope:
                    if self._is_visible(x, y):
                        if self.seethrough[x][y]:
                            if self._test_tile(x, y - 1, False):
                                startslope = self._get_inv_slope(x - .5,
                                                                 y - .5,
                                                                 self.sourcex,
                                                                 self.sourcey)
                        else:
                            if self._test_tile(x, y - 1, True):
                                self._scan(depth + 1, octant, startslope,
                                           self._get_inv_slope(x + .5, y - .5,
                                                               self.sourcex,
                                                               self.sourcey))
                        self.visiblemap[x][y] = True
                    y += 1
                y -= 1
        if x < 0:
            x = 0
        if x >= self.width:
            x = self.width - 1
        if y < 0:
            y = 0
        if y >= self.height:
            y = self.height - 1
        if self._is_visible(x, y) and self.seethrough[x][y]:
            self._scan(depth + 1, octant, startslope, endslope)

    def _check_bounds(self, x, y):
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            return False
        else:
            return True

    def _get_slope(self, x1, y1, x2, y2):
        return float(x1 - x2) / float(y1 - y2)

    def _get_inv_slope(self, x1, y1, x2, y2):
        return float(y1 - y2) / float(x1 - x2)

    def _is_visible(self, x, y):
        if self._check_bounds(self.sourcex, self.sourcey) and \
                self._check_bounds(x, y):
            return math.hypot(self.sourcex - x, self.sourcey - y) <= self.range
        else:
            return False

    def _test_tile(self, x, y, state):
        if not self._is_visible(x, y):
            return False
        else:
            return self.seethrough[x][y] == state
