/*
Assignment 3
Ben Malen
bm365

This assignment involves an extension to the single source/single destination
shortest path problem.
*/
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip> // setw
#include <math.h> // sqrt, pow
using namespace std;

const bool EXIT_WHEN_FOUND = false;
const char FILE_NAME[9] = "ass3.txt";
const unsigned int MAX = 50;

/*----------------------------------------------------------------------------*/
// Some helper functions
char indexToLabel(int i) { return (char)i + 'a'; }
unsigned int labelToIndex(char c) { return c - 'a'; }

/*----------------------------------------------------------------------------*/
// Declarations

struct Coord {
    Coord() {}
    int x_;
    int y_;
};

struct WeightedVertex {
    WeightedVertex(int vertex, double weight) : vertex_(vertex), weight_(weight) {}
    int vertex_;
    double weight_;
};
inline bool operator<(const WeightedVertex &lhs, const WeightedVertex &rhs) { return lhs.weight_ < rhs.weight_; }

class WeightedVertexQueue {
    public:
        WeightedVertexQueue() { nElements_ = 0; }
        bool isFull() { return nElements_ == MAX; }
        bool isEmpty() { return nElements_ == 0; }
        void enqueue(WeightedVertex *elements_);
        WeightedVertex * dequeue();
        void swapVertex(WeightedVertex *&a, WeightedVertex *&b);
        void siftUp(unsigned int i);
        void siftDown(unsigned int i);
        void clear();
    private:
        WeightedVertex * elements_[MAX];
        unsigned int nElements_;
};

struct Path {
    Path(unsigned int start, unsigned int goal, unsigned int nVertices);
    static void printPath(const Path &path, int vertex);
    static void print(const Path &path, unsigned int total = 0);
    static void printAll(const Path &path);
    int dist_[MAX], parent_[MAX];
    bool visited_[MAX];
    unsigned int nVisited_, start_, goal_, nVertices_;
};

struct Graph {
    Graph(unsigned int nVertices, unsigned int nEdges);
    void addEdge(unsigned int v1, unsigned int v2, unsigned int weight) { matrix_[v1][v2] = weight; }
    void deleteEdge(unsigned int v1, unsigned int v2) { matrix_[v1][v2] = -1; }
    void print();
    double euclideanDist(unsigned int v);
    void dijkstra(Path &path, bool aStar = false);
    void addCoord(unsigned int vertex, int x, int y) { coords_[vertex].x_ = x; coords_[vertex].y_ = y; };
    unsigned int secondShortest(Path &path, const Path &best, bool aStar = false);
    int matrix_[MAX][MAX];
    Coord coords_[MAX];
    double eDist_[MAX];
    unsigned int nVertices_, nEdges_, start_, goal_;
};

/*----------------------------------------------------------------------------*/
// Driver

int main() {
    ifstream fin;
    fin.open(FILE_NAME);
    if (!fin) {
        cerr << "Could not open " << FILE_NAME << endl;
        return 1;
    }
    // Read in number of vertices and edges
    unsigned int nVertices, nEdges;
    fin >> nVertices >> nEdges;
    if (nVertices > MAX) {
        cerr << "Number of vertices exceeds MAX constant." << endl;
        return 1;
    }
    Graph graph(nVertices, nEdges);
    // Read in coordinates of vertices
    char v;
    int x, y;
    for (unsigned int i = 0; i < nVertices; ++i) {
        fin >> v >> x >> y;
        graph.addCoord(labelToIndex(v), x, y);
    }
    // Read in edges and weights
    char v1Label, v2Label;
    unsigned int v1, v2, weight;
    for (unsigned int i = 0; i < nEdges; ++i) {
        fin >> v1Label >> v2Label >> weight;
        v1 = labelToIndex(v1Label);
        v2 = labelToIndex(v2Label);
        graph.addEdge(v1, v2, weight);
    }
    char startLabel, goalLabel;
    fin >> startLabel >> goalLabel;
    graph.start_ = labelToIndex(startLabel);
    graph.goal_ = labelToIndex(goalLabel);
    // Print graph
    graph.print();
    // Run the shortest path algorithms
    unsigned int total;
    cout << "\nShortest path using Dijkstra alg:" << endl;
    Path a1(graph.start_, graph.goal_, nVertices);
    graph.dijkstra(a1);
    Path::print(a1);
    Path::printAll(a1);
    cout << "\nSecond shortest path using Dijkstra alg:" << endl;
    Path a2(graph.start_, graph.goal_, nVertices);
    total = graph.secondShortest(a2, a1);
    Path::print(a2, total);
    Path::printAll(a2);
    cout << "\nShortest path using A* alg:" << endl;
    Path b1(graph.start_, graph.goal_, nVertices);
    graph.dijkstra(b1, true);
    Path::print(b1);
    Path::printAll(b1);
    cout << "\nSecond shortest path using A* alg:" << endl;
    Path b2(graph.start_, graph.goal_, nVertices);
    total = graph.secondShortest(b2, b1, true);
    Path::print(b2, total);
    Path::printAll(b2);
    return 0;
}

/*----------------------------------------------------------------------------*/
// WeightedVertexQueue implementations

// Inserts a new element into the heap.
// The new element is placed at the end of the heap and siftUp moves it up into the correct position.
void WeightedVertexQueue::enqueue(WeightedVertex *vertex) {
    if (isFull()) {
        cerr << "Enqueue overflow: MAX constant exceeded." << endl;
        exit(1);
    }
    elements_[nElements_++] = vertex;
    siftUp(nElements_ - 1);
}

// Removes the top element in the heap and returns pointer to that element, or NULL if queue is empty.
// The top element is replaced with the last element in the heap,
// and siftDown then moves that element back to the bottom of the heap.
WeightedVertex * WeightedVertexQueue::dequeue() {
    if (isEmpty())
        return NULL;
    WeightedVertex *pt = elements_[0];
    elements_[0] = elements_[nElements_ - 1];
    --nElements_;
    siftDown(0);
    return pt;
}

// Swap the addresses that the pointers are pointing to using reference-to-pointer.
void WeightedVertexQueue::swapVertex(WeightedVertex *&a, WeightedVertex *&b) {
    WeightedVertex *temp = a;
    a = b;
    b = temp;
}

// Min heap
// Moves element up to its correct position.
void WeightedVertexQueue::siftUp(unsigned int i) {
    if (i == 0) // then the element is the root
        return;
    unsigned int p = (i - 1) / 2; // integer division to find the parent
    if (*elements_[p] < *elements_[i]) // parent is smaller, so we will leave it as is
        return;
    else {
        swapVertex(elements_[i], elements_[p]); // put smallest in parent
        siftUp(p); // and siftUp parent
    }
}

// Min heap
// Moves element down to its correct position.
void WeightedVertexQueue::siftDown(unsigned int i) {
    unsigned int left = i * 2 + 1; // index of the left child
    if (left >= nElements_)
        return; // left child does not exist
    unsigned int smallest = left;
    unsigned int right = left + 1; // index of the right child
    if (right < nElements_) // right child exists
        if (*elements_[right] < *elements_[smallest]) // right child is smallest child
            smallest = right;
    if (*elements_[smallest] < *elements_[i]) {
        swapVertex(elements_[i], elements_[smallest]);
        siftDown(smallest);
    }
}

// Cleans up memory when we are done with the queue.
void WeightedVertexQueue::clear() {
    for (unsigned int i = 0; i < nElements_; ++i)
        delete elements_[i];
    nElements_ = 0;
}

/*----------------------------------------------------------------------------*/
// Path implementations

// Constructor
Path::Path(unsigned int start, unsigned int goal, unsigned int nVertices) : start_(start), goal_(goal), nVertices_(nVertices) {
    // Initialise arrays
    for (unsigned int i = 0; i < nVertices_; ++i) {
        dist_[i] = INT_MAX; // Infinity
        parent_[i] = -1;
        visited_[i] = false;
    }
    nVisited_ = 1;
}

// Static function that prints the vertices within a path
void Path::printPath(const Path &path, int vertex) {
    if (path.parent_[vertex] == -1)
        return;
    printPath(path, path.parent_[vertex]);
    cout << ", " << indexToLabel(vertex);
}

// Static function that prints information about the path
void Path::print(const Path &path, unsigned int total) {
    if (path.dist_[path.goal_] == INT_MAX) {
        cout << "No path found." << endl;
        return;
    }
    cout << "Path: " << indexToLabel(path.start_);
    printPath(path, path.goal_);
    cout << "\nPath distance: " << path.dist_[path.goal_]
         << "\nNumber of vertices visited: " << (total ? total : path.nVisited_) << endl;
}

void Path::printAll(const Path &path) {
    cout << endl;
    cout << setw(10) << "Vertex" << setw(10) << "Distance" << setw(10) << "Path" << endl;
    for (unsigned int i = 0; i < path.nVertices_; ++i) {
        cout << setw(10) << indexToLabel(i);
        if (path.dist_[i] == INT_MAX)
            cout << setw(10) << "INF";
        else
            cout << setw(10) << path.dist_[i];
        cout << setw(3) << indexToLabel(path.start_);
        printPath(path, i);
        cout << (path.visited_[i] ? "" : "*") << endl;
    }
}

/*----------------------------------------------------------------------------*/
// Graph implementations

// Constructor
Graph::Graph(unsigned int nVertices, unsigned int nEdges) :
    nVertices_(nVertices), nEdges_(nEdges) {
    // Initialise all weights and Euclidean distances to -1
    for (unsigned int i = 0; i < nVertices; ++i) {
        for (unsigned int j = 0; j < nVertices; ++j)
            matrix_[i][j] = -1;
        eDist_[i] = -1;
    }
}

// Prints out the entire matrix (not needed for this assignment, but included anyway).
void Graph::print() {
    cout << setw(4) << "";
    for (unsigned int i = 0; i < nVertices_; ++i)
        cout << setw(4) << indexToLabel(i);
    cout << endl;
    for (unsigned int i = 0; i < nVertices_; ++i) {
        cout << setw(4) << indexToLabel(i);
        for (unsigned int j = 0; j < nVertices_; ++j) {
            if (matrix_[i][j] != -1)
                cout << setw(4) << matrix_[i][j];
            else
                cout << setw(4) << '.';
        }
        cout << endl;
    }
    cout << endl;
}

// Returns the Euclidean distance from vertex, v, to the goal vertex.
// If the Euclidean distance has been previously calculated, it is fetched from an array.
double Graph::euclideanDist(unsigned int v) {
    if (eDist_[v] == -1)
        eDist_[v] = sqrt(pow(coords_[goal_].x_ - coords_[v].x_, 2) + pow(coords_[goal_].y_ - coords_[v].y_, 2));
    return eDist_[v];
}

// Dijkstra's algorithm
void Graph::dijkstra(Path &path, bool aStar) {
    cout << "\nDijkstra's algorithm called." << endl;
    WeightedVertexQueue pq; // Priority queue
    // We can set some values directly from the adjacency matrix (i.e. neighbouring vertices of starting vertex).
    for (unsigned int i = 0; i < nVertices_; ++i) {
        if (matrix_[start_][i] != -1) { // There is an edge
            path.dist_[i] = matrix_[start_][i];
            path.parent_[i] = start_;
            pq.enqueue(new WeightedVertex(i, path.dist_[i] + (aStar ? euclideanDist(i) : 0)));
        }
    }
    // Set the distance to 0 for the starting vertex and add it to the selected set
    path.dist_[start_] = 0;
    path.visited_[start_] = true;
    cout << indexToLabel(start_) << " selected with weight of " << path.dist_[start_] << "." << endl;
    while (!pq.isEmpty()) {
        WeightedVertex *current = pq.dequeue(); // Extract vertex with smallest weight
        unsigned int u = current->vertex_, weight = current->weight_;
        delete current;
        if (path.visited_[u])  // The shortest path has already been found for this vertex
            continue;
        cout << indexToLabel(u) << " selected with weight of " << weight << "." << endl;
        path.visited_[u] = true; // Final shortest distance from starting vertex has been determined
        ++path.nVisited_;
        // We can exit as soon as the shortest path to the goal has been found.
        if (path.visited_[path.goal_] && EXIT_WHEN_FOUND) {
            cout << "Goal found.\n" << endl;
            break;
        }
        // For each neighbour vertex, v, of u
        for (unsigned int v = 0; v < nVertices_; ++v) {
            if (matrix_[u][v] != -1 // if there is a connecting edge
                    && !path.visited_[v] // and the vertex is unvisited
                    && (path.dist_[u] + matrix_[u][v]) < path.dist_[v]) { // and there is a shorter path to v from the starting vertex, through u
                path.dist_[v] = path.dist_[u] + matrix_[u][v];
                path.parent_[v] = u;
                pq.enqueue(new WeightedVertex(v, path.dist_[v] + (aStar ? euclideanDist(v) : 0)));
            }
        }
    }
    pq.clear(); // Clean up memory
}

// Find the second shortest path
unsigned int Graph::secondShortest(Path &path, const Path &best, bool aStar) {
    int total = 0;
    path.dist_[goal_] = INT_MAX; // Initialise as infinity
    unsigned int current = goal_;
    // Loop through vertices, starting at the goal vertex, until we reach the starting vertex
    while (current != start_) {
        // Store the edge info so we can restore it later
        unsigned int v1 = best.parent_[current], v2 = current, weight = matrix_[v1][v2];
        deleteEdge(v1, v2); // Delete the edge
        Path candidate(start_, goal_, nVertices_);
        dijkstra(candidate, aStar); // Run Dijkstra's algorithm, without the edge
        // Add to the total number of vertices visited
        total += candidate.nVisited_;
        if (candidate.dist_[goal_] != INT_MAX) { // If the goal is reachable
            // Update the 2nd shortest path with the candidate if it is shorter
            if (candidate.dist_[goal_] < path.dist_[goal_])
                path = candidate;
        }
        current = best.parent_[current];
        // Restore the deleted edge
        addEdge(v1, v2, weight);
    }
    return total;
}

/*------------------------------------------------------------------------------

The graph is read into an array representing an adjacency matrix. Coordinates of
vertices are read into an array of Coord objects. Path objects are used to store
the resulting path information from the shortest path algorithms.

At the start of Dijkstra's algorithm, some values are set directly from the
adjacency matrix (i.e. neighbouring vertices of starting vertex). The algorithm
will stop as soon as the shortest path to the goal has been found. The algorithm
uses a priority queue (min-heap) of pointers to WeightedVertexQueue objects.
Dequeue will always pick the vertex with the minimum weight. Vertices are
weighted by their distance from the starting vertex. When A* is used, the weight
is this distance plus the Euclidean distance between the current vertex and
goal. If the Euclidean distance has been previously calculated, it is fetched
from an array.

The proposed solution from the assignment specification is used to find the
second shortest path.

Notes:
The "second shortest path" algorithm uses the path found by the shortest path
algorithm. The "Number of vertices visited" does not include those counted to
find this path.

What if we require that the second shortest path be longer than the shortest
path?

Since the proposed solution may find an alternate route with the same distance,
Dijkstra's algorithm would need to be modified to find k-shortest paths, then
k+1 shortest paths may be found until the path's distance is greater than the
shortest. Eppstein's algorithm or Yen's algorithm can also be used to find the
k-shortest paths.

What if the graph contains cycles?

If the graph contains cycles, the second shortest path may contain the shortest
path. If an edge is removed, the algorithm would fail to find this path.
A different approach is needed, such as Eppstein's algorithm. Cycles can be
detected if a depth-first search finds an edge that points back to an ancestor
of the current vertex (back edge). Eppstein's algorithm finds all possible
deviations from the shortest path, from which k-shortest paths are obtained.

What if the graph is undirected?

Similarly, if it's an undirected graph, the second shorted path may contain
the shortest path, and if an edge is removed, the algorithm would fail to find
this path. The same approach mentioned for cycles can be applied.

------------------------------------------------------------------------------*/
