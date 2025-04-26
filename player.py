import math
class Player():
    def __init__(self, direction):
        self.direction = direction
        self.position = [0,0,0]
        self.speed = 100
        self.reply = False
        self.g = 9.81
        self.d = 0.97
    def setposition(self, pos):
        self.position = pos
    def getposition(self):
        return(self.position)
    def getreply(self):
        return(self.reply)
    def updateposition(self, bpos, bvel):
        if(bpos[1]<0 or bvel[1]<0):
            self.reply=False
        else:
            if(bpos[2] < 1):
                self.reply=True
        
        if((bvel[1]*self.direction > 0) and (bpos[1]*self.direction >0)):
            xf = 0
            yf = -30 * self.direction
            self.position[0] = self.position[0] + (xf - self.position[0]) / self.speed
            self.position[1] = self.position[1] + (yf - self.position[1]) / self.speed
        else:
            if(bvel[1]*self.direction < -0.1):
                sr = bvel[2]*bvel[2] + 2*self.g* (bpos[2]-0.8)
                if(sr > 0):
                    tf = (bvel[2] + math.sqrt(sr)) / self.g
                    if(tf < 0):
                        tf = (bvel[2] - math.sqrt(sr)) / self.g
                    xf = bpos[0] + bvel[0] * (1 - math.exp(-self.d*tf))/self.d
                    yf = bpos[1] + bvel[1] * (1 - math.exp(-self.d*tf))/self.d
                    #add speed
                    xf = xf + bvel[0]/5
                    yf = yf + bvel[1]/5
                    if((bvel[2]<0 and bpos[2]>1) or bpos[1]*self.direction>0):
                        self.position[0] = self.position[0] + (xf - self.position[0]) / self.speed
                        self.position[1] = self.position[1] + (yf - self.position[1]) / self.speed
        return(self.position)
    