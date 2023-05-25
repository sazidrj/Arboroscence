#include<bits/stdc++.h>
using namespace std;

struct Edge{
    int u;
    int v;
    int real_weight;
    int dummy_weight;
    pair<int,int> original_nodes = {0,0};

    Edge(int u, int v, int d) {
        this->u = u;
        this->v = v;
        this->real_weight = d;
        this->dummy_weight = d;
    }
};

struct customComparator{
    bool operator()(const Edge e1, const Edge e2){

        int d1 = e1.dummy_weight;
        int d2 = e2.dummy_weight;

        return d1 > d2;
    }
};


class Graph {
    vector<vector<Edge *>> graph;
    // vector<pair<int, Edge*>> min_edge_weight;
    vector<int> min_edge_weight;
    unordered_map<int, Graph*> collapseNodes;
    int V;

public:

    Graph(int V) {
        this->V = V;
        graph.resize(V, vector<Edge *>(V));
        min_edge_weight.resize(V);
    }

    void addEdge(int u, int v, int d) {
        Edge *e = new Edge(u, v, d);
        graph[u][v] = e;
    }

    // Finding minimum edge weight of each vertex
    void cal_min_edge_weight(){

        for(int j = 1; j<V; j++){
            Edge *e = NULL;
            for(int i = 1; i<V; i++){
                if(graph[i][j] != NULL){
                    if(e == NULL){
                        e = graph[i][j];
                    }else if(e->dummy_weight > graph[i][j]->dummy_weight){
                        e = graph[i][j];
                    }
                }
            }

            if(e == NULL){
                min_edge_weight[j] = INT_MAX;
            }else{
                min_edge_weight[j] = e->dummy_weight;
            }

        }
    }

    // Subtract min_edge_weight from every incoming edge of all vertices
    void decrease_min_edge_weight(){
        for(int i = 1; i<V; i++){
            // pair<int,Edge*> p = min_edge_weight[i];
            for(int j = 1; j<V; j++){
                if(graph[j][i] != NULL){
                    graph[j][i]->dummy_weight -= min_edge_weight[i];
                }
            }
        }
    }

    void displayGraph() {
        for (int i = 1; i < V; i++) {
            cout << "Vertex = " << i << " : ";
            for (int j = 1; j < V; j++) {
                if(graph[i][j] != NULL)
                    cout <<"("<<j<<","<<graph[i][j]->dummy_weight<<")";
            }
            cout << endl;
        }
        cout<<endl;
    }

    // Function to return collapsed graph
    pair<Graph, vector<int>> collapseGraph(Graph *g, vector<bool> nodesPresentInCycle, int source){

        int countCycleNodes = 0;
        for(int i = 0; i<nodesPresentInCycle.size(); i++){
            if(nodesPresentInCycle[i]){
                countCycleNodes++;
            }
        }

        int V = g->V;
        int newV = V-countCycleNodes+1;
        Graph newGraph(newV);

        collapseNodes[newV-1] = g;

        int count = 1;
        vector<int> nodesMapping(V,-1);
        for(int i = 1; i<nodesPresentInCycle.size(); i++){
            if(!nodesPresentInCycle[i])
                nodesMapping[i] = count++;
            else
                nodesMapping[i] = newV-1;
        }

        for(int i = 1; i<V; i++){
            for(int j = 1; j<V;j++){
                if(g->graph[i][j] != NULL){
                    if(nodesMapping[i] != nodesMapping[j]){
                        if(newGraph.graph[nodesMapping[i]][nodesMapping[j]] == NULL){
                            newGraph.addEdge(nodesMapping[i], nodesMapping[j], g->graph[i][j]->dummy_weight);
                            newGraph.graph[nodesMapping[i]][nodesMapping[j]]->original_nodes.first = i;
                            newGraph.graph[nodesMapping[i]][nodesMapping[j]]->original_nodes.second = j;
                        }else if(g->graph[i][j]->dummy_weight < newGraph.graph[nodesMapping[i]][nodesMapping[j]]->dummy_weight){
                            newGraph.addEdge(nodesMapping[i], nodesMapping[j], g->graph[i][j]->dummy_weight);
                            newGraph.graph[nodesMapping[i]][nodesMapping[j]]->original_nodes.first = i;
                            newGraph.graph[nodesMapping[i]][nodesMapping[j]]->original_nodes.second = j;
                        }
                    }
                }
            }
        }

        nodesMapping[0] = nodesMapping[source];

        return {newGraph,nodesMapping};

    }

    // Function to return expanded graph
    Graph* expandGraph(Graph *g, vector<int> nodesMapping){
        int V = g->V;
        Graph *prevGraph;
        for(int i = 1; i<V; i++){
            if(collapseNodes.count(i) > 0){
                prevGraph = collapseNodes[i];
                collapseNodes.erase(i);
                break;
            }
        }

        int prevV = prevGraph->V;

        vector<bool> coveredEdges(prevV, false);

        for(int i = 1; i<V; i++){
            for(int j = 1; j<V; j++){
                if(g->graph[i][j] != NULL){
                    Edge *tempEdge = g->graph[i][j];
                    int from_node = tempEdge->original_nodes.first;
                    int to_node = tempEdge->original_nodes.second;

                    coveredEdges[to_node] = true;

                    for(int k = 1; k<prevV; k++){
                        if(k != from_node){
                            if(prevGraph->graph[k][to_node] != NULL){
                                prevGraph->graph[k][to_node] = NULL;
                            }
                        }
                    }
                }
            }
        }


        for(int i = 1; i<prevV; i++){
            bool flag = false;
            for(int j = 1; j<prevV; j++){
                if(coveredEdges[i]){
                    continue;
                }

                if(prevGraph->graph[j][i] != NULL){
                    if(prevGraph->graph[j][i]->dummy_weight != 0){
                        prevGraph->graph[j][i] = NULL;
                    }else if(prevGraph->graph[j][i]->dummy_weight == 0){
                        if(flag == true){
                            prevGraph->graph[j][i] = NULL;
                        }else{
                            flag = true;
                        }
                    }
                }
            }
        }

        return prevGraph;
    }

    // DFS detect the cycle present
    bool dfs(int node, vector<bool> &visited, vector<bool> &present_in_stack, vector<bool> &present_in_cycle){
        visited[node] = true;
        present_in_stack[node] = true;

        for(int adjNode = 1; adjNode<V; adjNode++){
            if(graph[node][adjNode] != NULL && graph[node][adjNode]->dummy_weight == 0){
                if(present_in_stack[adjNode] == true){
                    present_in_cycle[adjNode] = true;
                    return true;
                }else if(visited[adjNode] == false && graph[node][adjNode]->dummy_weight == 0){
                    bool adjNodeFoundCycle =  dfs(adjNode,visited,present_in_stack, present_in_cycle);
                    if(adjNodeFoundCycle){
                        present_in_cycle[adjNode] = true;
                        return adjNodeFoundCycle;
                    }
                }
            }
        }

        present_in_stack[node] = false;

        return false;
    }

    // Function to detect cycle in the graph which call dfs on every node
    pair<bool,vector<bool>> detect_cycle(){
        //   Graph g2 = modifiedGraph(g);

        // int V = g2.V;

        vector<bool> visited(V, false);
        vector<bool> present_in_stack(V, false);
        vector<bool> present_in_cycle(V, false);

        for(int i = 1; i<V; i++){
            if(!visited[i]){
                bool foundCycle = dfs(i,visited,present_in_stack, present_in_cycle);
                if(foundCycle == true)
                    return {foundCycle, present_in_cycle};
            }
        }
        return {false, present_in_cycle};
    }

    Graph* mst(Graph *g, int src){

        priority_queue<Edge, vector<Edge>, customComparator> pq;

        int V = g->V;

        vector<bool> visited(V,false);
        visited[src] = true;

        for(int i = 1; i<V; i++){
            if(g->graph[src][i] != NULL){
                pq.push(*(g->graph[src][i]));
            }
        }

        while(!pq.empty()){
            Edge minEdge = pq.top();
            pq.pop();

            if(visited[minEdge.v]){
                g->graph[minEdge.u][minEdge.v] = NULL;
            }else{
                visited[minEdge.v] = true;
                for(int i = 1; i<V; i++){
                    if(g->graph[minEdge.v][i] != NULL){
                        pq.push(*(g->graph[minEdge.v][i]));
                    }
                }
            }
        }
        return g;
    }

    // Minimum Arboroscene recursive function
    Graph* min_arboroscence_rec(Graph *g, int source){
        g->cal_min_edge_weight();
        g->decrease_min_edge_weight();

        pair<bool,vector<bool>> cycle_present = g->detect_cycle();
        if(!cycle_present.first){
            return mst(g, source);
        }else{
            pair<Graph, vector<int>> temp = collapseGraph(g,cycle_present.second, source);
            Graph colGraph = temp.first;
            vector<int> nodesMapping = temp.second;

            Graph *mstGraph =  min_arboroscence_rec(&colGraph, nodesMapping[0]);

            return expandGraph(mstGraph, nodesMapping);

        }
    }

    void dfs2(int source, vector<bool> &visited, vector<int> &par, vector<int> &cost, int &min_arbo_cost){
        visited[source] = true;

        for(int i = 1; i<V; i++){
            if(graph[source][i] != NULL && !visited[i]){
                par[i] = source;
                if(cost[i] == -1){
                    cost[i] = 0;
                }
                cost[i] = cost[source] + graph[source][i]->real_weight;
                min_arbo_cost += graph[source][i]->real_weight;
                dfs2(i,visited,par,cost, min_arbo_cost);
            }
        }
    }

    void outputFunction(int source){
        vector<bool> visited(V, false);
        vector<int> par(V, 0);
        vector<int> cost(V,-1);
        int min_arbo_cost = 0;
        cost[source] = 0;
        par[source] = 0;

        dfs2(source,visited, par, cost, min_arbo_cost);

        cout<<min_arbo_cost<<" ";

        for(int i = 1; i<V; i++){
            cout<<cost[i]<<" ";
        }
        cout<<"#"<<" ";
        for(int i = 1; i<V; i++){
            cout<<par[i]<<" ";
        }
        cout<<endl;
    }

    // Main min_arboroscene function which use helper function min_arboroscence which run recursively
    void min_arboroscence(int source){
        // displayGraph();
        Graph *mst =  min_arboroscence_rec(this, source);
        // (*mst).displayGraph();

        (*mst).outputFunction(source);



    }
};



int main(){

    int t;
    cin>>t;

    while(t--) {
        int V, M, S;
        cin >> V >> M >> S;

        Graph g(V+1);
        for (int i = 0; i < M; i++) {
            int u, v, d;
            cin >> u >> v >> d;
            g.addEdge(u, v, d);
        }

        g.min_arboroscence(S);
    }


    return 0;
}