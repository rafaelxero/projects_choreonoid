from cnoid.Base import *
from cnoid.BodyPlugin import *

ball = Item.find("ball").body().rootLink()
floor1 = Item.find("floor1").body().rootLink()
floor2 = Item.find("floor2").body().rootLink()
box = Item.find("box").body().rootLink()
simulator = Item.find("AISTSimulator")

simulator.setFriction(ball, floor1, 0.5, 0.5)
#simulator.setEpsilon(ball, floor1, 0.5)

simulator.setFriction(ball, floor2, 0.5, 0.5)
#simulator.setEpsilon(ball, floor2, 0.75)

simulator.setFriction(ball, box, 0.5, 0.5)
#simulator.setEpsilon(ball, box, 1.0)
