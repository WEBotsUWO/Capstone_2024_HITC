#treeNode class

class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.value = [locationX, locationY] # we need to store their position in space to check for collisions.
        self.children = [] # parent can have many children
        self.parent = None # each node can only have one parent
    