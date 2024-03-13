class Tree:
    def __init__(self,root):
        self.root = root
    def GetClosestNode(self,pos):
        return self.root.GetClosestNodeInChildren(pos)
class Edge:
    def __init__(self, n_p, n_c):
        self.parent = n_p
        self.child = n_c
        self.flag = "VALID"