class LineTracker:
    def __init__(self, maxDisappeared=10):
        self.id = 0
        self.line = None
        self.disappeared = 0
        self.maxDisappeared = maxDisappeared

    def add(self, line):
        self.line = line
        self.disappeared = 0
        self.id += 1

    def remove(self):
        self.line = None
        self.disappeared = 0

    def update(self, line):
        if line is None:
            self.disappeared += 1

            if self.disappeared > self.maxDisappeared:
                self.remove()
        else:
            if self.line is None:
                self.add(line)
            else:
                distance = self.distance(line, self.line)
                if distance < 75:
                    self.line = line
                    self.disappeared = 0
                else:
                    self.add(line)

    def distance(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        
        midPoint1 = (x1 + x2) / 2, (y1 + y2) / 2
        midPoint2 = (x3 + x4) / 2, (y3 + y4) / 2

        return ((midPoint2[0] - midPoint1[0]) ** 2 + (midPoint2[1] - midPoint1[1]) ** 2) ** 0.5
    