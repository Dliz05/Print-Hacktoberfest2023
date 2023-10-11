import java.io.*;
import java.util.*;

public class Main {

  static class Edge {

    int src; // One of the nodes
    int nbr; // The second node forming the edge
    int wt; // Weight of the edge

    Edge(int src, int nbr, int wt) {
      this.src = src;
      this.nbr = nbr;
      this.wt = wt;
    }
  }

  static class Pair implements Comparable<Pair> {

    int vtx; // The vertex
    String psf; // The path from source node
    int wsf; // THe weight of the path
    
    Pair(int vtx, String psf, int wsf) {
      this.vtx = vtx;
      this.psf = psf;
      this.wsf = wsf;
    }
    
    // To compare two paths/routes, just compare their weights
    public int compareTo(Pair o) {
      return this.wsf - o.wsf;
    }
  }

  public static void main(String[] args) throws Exception {
    BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
    
    // Read the number of vertices.
    int vtces = Integer.parseInt(br.readLine());
    ArrayList<Edge>[] graph = new ArrayList[vtces + 1];
      
    for (int i = 0; i < vtces + 1; i++) {
      graph[i] = new ArrayList<>();
    }
    
    // Read the number of edges edges
    int edges = Integer.parseInt(br.readLine());
      
    // Read the details of each of the edge.
    for (int i = 0; i < edges; i++) {
      String[] parts = br.readLine().split(" ");
      int v1 = Integer.parseInt(parts[0]);
      int v2 = Integer.parseInt(parts[1]);
      int wt = Integer.parseInt(parts[2]);
      graph[v1].add(new Edge(v1, v2, wt));
      graph[v2].add(new Edge(v2, v1, wt));
    }
    
    // Input the source vertex
    int src = Integer.parseInt(br.readLine());
    boolean[] visited = new boolean[vtces + 1];
      
    // Priority Queue to hold the pairs.
    PriorityQueue<Pair> pq = new PriorityQueue<>();
    pq.add(new Pair(src, src + "", 0));
      
    // Iterate till the priority queue is not empty
    while (pq.size() > 0) {
        
      // Remove the top vertex or the lowest path end point
      Pair rem = pq.remove();
      if (visited[rem.vtx] == true) {
        continue;
      }
        
      // If the vertex is not visited, mark it visited as 
      // the shortest path to the vertex is found.
      visited[rem.vtx] = true;
      System.out.println(rem.vtx + " via " + rem.psf + " @ " + rem.wsf);

      // For each adjacent node of the vertex,
      // Relax the distance to the adjacent edge
      for (Edge e : graph[rem.vtx]) {
        if (visited[e.nbr] == false) {
          pq.add(new Pair(e.nbr, rem.psf + "->" + e.nbr, rem.wsf + e.wt));
        }
      }
    }
  }
}




Dijkstra shortest path algorithm for Adjacency Matrix in O(V2):
The idea is to generate a SPT (shortest path tree) with a given source as a root. Maintain an Adjacency Matrix with two sets, 

one set contains vertices included in the shortest-path tree, 
other set includes vertices not yet included in the shortest-path tree. 
At every step of the algorithm, find a vertex that is in the other set (set not yet included) and has a minimum distance from the source.

// A Java program for Dijkstra's single source shortest path
// algorithm. The program is for adjacency matrix
// representation of the graph
import java.io.*;
import java.lang.*;
import java.util.*;

class ShortestPath {
	// A utility function to find the vertex with minimum
	// distance value, from the set of vertices not yet
	// included in shortest path tree
	static final int V = 9;
	int minDistance(int dist[], Boolean sptSet[])
	{
		// Initialize min value
		int min = Integer.MAX_VALUE, min_index = -1;

		for (int v = 0; v < V; v++)
			if (sptSet[v] == false && dist[v] <= min) {
				min = dist[v];
				min_index = v;
			}

		return min_index;
	}

	// A utility function to print the constructed distance
	// array
	void printSolution(int dist[])
	{
		System.out.println(
			"Vertex \t\t Distance from Source");
		for (int i = 0; i < V; i++)
			System.out.println(i + " \t\t " + dist[i]);
	}

	// Function that implements Dijkstra's single source
	// shortest path algorithm for a graph represented using
	// adjacency matrix representation
	void dijkstra(int graph[][], int src)
	{
		int dist[] = new int[V]; // The output array.
								// dist[i] will hold
		// the shortest distance from src to i

		// sptSet[i] will true if vertex i is included in
		// shortest path tree or shortest distance from src
		// to i is finalized
		Boolean sptSet[] = new Boolean[V];

		// Initialize all distances as INFINITE and stpSet[]
		// as false
		for (int i = 0; i < V; i++) {
			dist[i] = Integer.MAX_VALUE;
			sptSet[i] = false;
		}

		// Distance of source vertex from itself is always 0
		dist[src] = 0;

		// Find shortest path for all vertices
		for (int count = 0; count < V - 1; count++) {
			// Pick the minimum distance vertex from the set
			// of vertices not yet processed. u is always
			// equal to src in first iteration.
			int u = minDistance(dist, sptSet);

			// Mark the picked vertex as processed
			sptSet[u] = true;

			// Update dist value of the adjacent vertices of
			// the picked vertex.
			for (int v = 0; v < V; v++)

				// Update dist[v] only if is not in sptSet,
				// there is an edge from u to v, and total
				// weight of path from src to v through u is
				// smaller than current value of dist[v]
				if (!sptSet[v] && graph[u][v] != 0
					&& dist[u] != Integer.MAX_VALUE
					&& dist[u] + graph[u][v] < dist[v])
					dist[v] = dist[u] + graph[u][v];
		}

		// print the constructed distance array
		printSolution(dist);
	}

	// Driver's code
	public static void main(String[] args)
	{
		/* Let us create the example graph discussed above
		*/
		int graph[][]
			= new int[][] { { 0, 4, 0, 0, 0, 0, 0, 8, 0 },
							{ 4, 0, 8, 0, 0, 0, 0, 11, 0 },
							{ 0, 8, 0, 7, 0, 4, 0, 0, 2 },
							{ 0, 0, 7, 0, 9, 14, 0, 0, 0 },
							{ 0, 0, 0, 9, 0, 10, 0, 0, 0 },
							{ 0, 0, 4, 14, 10, 0, 2, 0, 0 },
							{ 0, 0, 0, 0, 0, 2, 0, 1, 6 },
							{ 8, 11, 0, 0, 0, 0, 1, 0, 7 },
							{ 0, 0, 2, 0, 0, 0, 6, 7, 0 } };
		ShortestPath t = new ShortestPath();

		// Function call
		t.dijkstra(graph, 0);
	}
}
// This code is contributed by Aakash Hasija
