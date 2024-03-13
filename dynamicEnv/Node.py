class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None
        self.flag = "VALID"
    def position(self):
        return (self.x,self.y)
    def print(self):
        print(self.x,self.y)

    def path_vertical(self):
        if self.parent:
            return [self.y, self.parent.y]
        else:
            return []

    def path_horizontal(self):
        if self.parent:
            return [self.x, self.parent.x]
        else:
            return []
    def GetClosestNodeInChildren(self,target):
        closest_sqr_distance = sqr_magnitude(self,target)
        closest_node = self
        for child in self.children:
            temp_node = child.GetClosestNodeInChildren(target)
            temp_sqr_distance = sqr_magnitude(temp_node,target)
            if temp_sqr_distance < closest_sqr_distance:
                closest_sqr_distance = temp_sqr_distance
                closest_node = temp_node
        return closest_node,closest_sqr_distance
def sqr_magnitude(current,target):
    return (current.x - target.x) ** 2 + (current.y - target.y) ** 2