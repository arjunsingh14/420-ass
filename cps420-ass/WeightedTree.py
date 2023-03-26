from WeightedGraph import WeightedGraph

class WeightedTree(WeightedGraph):
    """
    WeightedTree objects are WeightedGraphs that are trees.
    They are internally represented using modified adjacency matrices where each entry a(i,j) is either
    - 0 if there is no edge between vertices i and j
    - a positive integer representing the weight of the edge between i and j if there is such an edge
    Note that the graph is simple and therefore the adjacency matrix representation is symmetric
    """

###########################  DO NOT MODIFY THESE FUNCTIONS  ####################################


    def __init__(self, vertices, edges):
        self.inPath = [0]*vertices # used to check for an existing path between 2 vertices
        super().__init__(vertices, edges)

    def __init__(self, vertices):
        self.inPath = [0]*vertices # used to check for an existing path between 2 vertices
        edges = []
        for i in range(vertices):
            edges.append(list([0]*vertices))
        super().__init__(vertices, edges)


    @classmethod
    def fromFile(cls, filename):
        """
        Instantiates a WeightedGraph read from a file.  
        See the description of WeightedGraph.readGraph for the file format.
        
        Parameters:
            str filename: name of file containing the graph
        
        Returns a WeightedGraph described by the file. 
        """
        vertices, edges = WeightedGraph.readGraph(filename)
        return WeightedTree(vertices, edges)

###########################  START YOUR CODE HERE  ####################################


    @classmethod
    def MSTfromGraph(cls, graph):
        """
              Creates a WeightedTree that is a MST of graph.  
        
        Parameters:
            WeightedGraph graph: graph whose MST will be computed
        
        Returns a WeightedTree MST for the graph, or None if this is not possible. 
              """
        edges = cls.sortEdges(graph)
        tree = WeightedTree(graph.totalVertices())
        for i in edges:
            tree.clearVisited()
            tree.addEdge(i)
            if tree.totalE == tree.totalV - 1:
                break
        if tree.isSpanningtree(graph):
            return tree
        return None


    @classmethod
    def sortEdges(cls, graph):
        """
              Sort the edges of the graph in order of increasing weight 
        
        Parameters:
            WeightedGraph graph: graph whose edges will be sorted
        
        Returns a sorted list of the edges of the graph.
        Each edge is a triple of format (weight, v1, v2) 
              """
        edges = []
        graphEdges = graph.edges;
        for i in range(graph.totalV):
            for j in range(graph.totalV):
                if graphEdges[i][j] != 0:
                    edges.append((graphEdges[i][j], i, j))

        return sorted(edges)

    def canAdd(self,newedge):
        """
        Checks whether a new edge can be added to self without introducing a cycle
        
        Parameters:
            triple newedge: edge that could be added.  Its format is (weight,v1,v2)
        
        Returns True if newedge can be added to self without introducing a cycle, and False otherwise
        """

        weight, i, j = newedge

        if self.edges[i][j] != 0:
            return False
        if self.isPath(i, j):
            return False
        return True




    def isPath(self,i,j):
        """
        Determines whether there is a path from i to j in self
        by trying to find such a path recursively, backtracking when necessary
        
        Parameters:
            int i,j: vertices in self which may or may not be connected
        
        Returns True if self contains a path from i to j, and False otherwise
        
        Side-Effect:
            self.inPath[] is used and modified by this method as follows (where v is a vertex):
            - self.inPath[v] = 0 when self does not include any edges ajacent to v yet
            - self.inPath[v] = 1 when self does include at least one edge adjacent to v,
                            but v is not yet part of the path  
            - self.inPath[v] = 2 when self does include at least one edge adjacent to v,
                            and v is already part of the path   
        """
        self.clearVisited()
        stack = [(i, -1)]
        while stack:
            curr, parent = stack.pop()
            self.visitedV[curr] = True

        # Check if current vertex is the end vertex
            if curr == j:
                return True

        # Add unvisited neighbors to the stack
            for neighbor, weight in enumerate(self.edges[curr]):
                if weight > 0:
                    if not self.visitedV[neighbor]:
                        stack.append((neighbor, curr))
                    elif neighbor != parent:
                    # If neighbor has been visited and is not the parent of the current vertex,
                    # then there is a cycle between the current vertex and neighbor
                        return True

    # If end vertex was not found or no cycles were detected, return False
        return False
    def addEdge(self,newedge):
        """
        Adds a new edge to self
        
        Parameters:
            triple newedge: edge that will be added.  Its format is (weight,v1,v2)
        
        Returns nothing
        """
        if self.canAdd(newedge):
            self.edges[newedge[1]][newedge[2]] = newedge[0]
            self.edges[newedge[2]][newedge[1]] = newedge[0]
            self.totalE += 1
            self.totalW += newedge[0]

###########################  COPY YOUR LAB6 CODE FOR THESE FUNCTIONS  ####################################
    def isTree(self):
        """
        Checks whether self is tree
        
        Returns True if self is a tree, and False otherwise
        """
        if self.totalVertices() - 1 ==  self.totalEdges() and self.isConnected():
            return True
        return False

    def isSpanningtree(self,graph):
        """
        Checks whether self is a spanning tree of a graph

        Parameters:
            int graph: WeightedGraph that may have self as a spanning tree

        Assumptions:
            the vertices have the same numbering in both graphs
            
        Returns True if self is a spanning tree of graph, and False otherwise
        """

        return self.isSubgraph(graph) and self.isTree()
