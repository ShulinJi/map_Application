/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cppFiles/file.cc to edit this template
 */

#include "m4.h"
#include "m3.h"
#include "m1.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <list>
#include <utility>
#include <algorithm>
#include <chrono>
#include <thread>
#define NO_EDGE -1
#define INFINITE 9999999999
//#define MAX_SPEED 33.333333

struct waveElem {
    IntersectionIdx nodeID;  // current node 
    StreetSegmentIdx edgeID; // edge to reach current node
    double travelTime;       // Total travel time to reach the node
    
    waveElem () {
        nodeID = 0; edgeID = 0; travelTime = 0.0;
    }
    
    waveElem (int node, int edge, double travel_time) {
        nodeID = node; edgeID = edge; travelTime = travel_time;
    }
    
    // overload the operator to give the smallest estimation time entry at the top of the heap 
    // heap was biggest at the top
    bool operator< (const waveElem & rhs) const {
        if (travelTime > rhs.travelTime) {
            return true;
        } else {
            return false;
        }
    }
};

struct matrix_element {
    double travel_time;
    std::vector<StreetSegmentIdx> path_to_dest;
    IntersectionIdx from;
    IntersectionIdx to;
};

// Class to store the bestTime and its reaching_edge that results the bestTime
class Node {
public:
    StreetSegmentIdx reachingEdge; // edge used to reach current node
    double bestTime = INFINITE; // shortest time to get this node 
};

struct intersection_info {
    IntersectionIdx intersection_id = 0;
    bool can_go;
    bool visited;
};

std::vector <intersection_info> pickup_dropoff_depot; // convert the struct into a vector that in order of pickup and dropoff
std::vector <std::vector<matrix_element>> matrix_travel_time;
//std::vector<Node> nodesM4; // Vector that stores the bestTime and reaching edge of each node
std::unordered_multimap<IntersectionIdx, int> intersect_to_matrix_index; 
bool path_pre_compute (IntersectionIdx srcID, const float turn_penalty, int num_of_matrix_rows);
double travelTimeM4 (StreetSegmentIdx outEdge);
void convert_struct_to_vector (const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots);
std::vector<StreetSegmentIdx> bfsTraceBackM4 (int destID, const std::vector<Node>& nodesM4);
void load_unordered_multimap (const std::vector<DeliveryInf>& deliveries,
                              const std::vector<IntersectionIdx>& depots);

double travelTimeM4 (StreetSegmentIdx outEdge) {
    return findStreetSegmentTravelTime (outEdge);
}

void load_unordered_multimap (const std::vector<DeliveryInf>& deliveries,
                              const std::vector<IntersectionIdx>& depots) {
    
    for (int num_dest = 0; num_dest < (deliveries.size() * 2); num_dest++) {
        IntersectionIdx curr_intersect = pickup_dropoff_depot[num_dest].intersection_id;
        for (int index_of_matrix = 0; index_of_matrix < pickup_dropoff_depot.size(); index_of_matrix++) {
            if (curr_intersect == pickup_dropoff_depot[index_of_matrix].intersection_id) {
                std::pair<IntersectionIdx, int> mypair (curr_intersect, index_of_matrix);
                intersect_to_matrix_index.insert(mypair);
            }
        }
    }
    
    for (int num_dest = 0; num_dest < depots.size(); num_dest++) {
        IntersectionIdx curr_intersect = depots[num_dest];
        for (int index_of_matrix = 0; index_of_matrix < depots.size(); index_of_matrix++) {
            if (curr_intersect == depots[index_of_matrix]) {
                std::pair<IntersectionIdx, int> mypair (curr_intersect, (index_of_matrix + deliveries.size() * 2));
                intersect_to_matrix_index.insert(mypair);
            }
        }
    }
}

bool path_pre_compute (IntersectionIdx srcID, const float turn_penalty, int num_rows) {
    //auto const start = std::chrono::high_resolution_clock::now();
    std::vector<Node> nodesM4;
    nodesM4.resize(getNumIntersections());
    std::priority_queue<waveElem> wavefront; // queue that is used to store the nodes that are going to be checked
    waveElem temp;
    temp.nodeID = srcID; temp.edgeID = NO_EDGE; temp.travelTime = 0;
    wavefront.push(temp);
    
    int count_of_dest_reached = 0;
    while (wavefront.size() != 0) {
        //auto const start1 = std::chrono::high_resolution_clock::now();
        waveElem curr = wavefront.top();
        wavefront.pop();
        
        if (curr.travelTime < nodesM4[curr.nodeID].bestTime) {
            nodesM4[curr.nodeID].reachingEdge = curr.edgeID;
            nodesM4[curr.nodeID].bestTime = curr.travelTime;
            
            if (intersect_to_matrix_index.find(curr.nodeID) != intersect_to_matrix_index.end()) {
                auto range = intersect_to_matrix_index.equal_range(curr.nodeID);
                for (auto it = range.first; it != range.second; it++) {
                    int index = it -> second;
                    matrix_travel_time[num_rows][index].travel_time = nodesM4[curr.nodeID].bestTime;
                    count_of_dest_reached++;
                }
            }
                    
            if (count_of_dest_reached == intersect_to_matrix_index.size()) {
//                auto const end = std::chrono::high_resolution_clock::now();
//                auto const delta_time_1 = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
//                std::cout << "One Multidijsktra cost " << delta_time_1.count() << "s\n";
                
            for (int num_of_dest = 0; num_of_dest < pickup_dropoff_depot.size(); num_of_dest++){
                
//                auto const start1 = std::chrono::high_resolution_clock::now();
                //load the struct element int the matrix 
                std::vector<StreetSegmentIdx> path = bfsTraceBackM4 (pickup_dropoff_depot[num_of_dest].intersection_id, nodesM4);
                matrix_travel_time[num_rows][num_of_dest].path_to_dest = path;
                //double path_time = computePathTravelTime (turn_penalty, path);
              
                //matrix_travel_time[num_rows][num_of_dest].travel_time = path_time;
                matrix_travel_time[num_rows][num_of_dest].from = pickup_dropoff_depot[num_rows].intersection_id;
                matrix_travel_time[num_rows][num_of_dest].to = pickup_dropoff_depot[num_of_dest].intersection_id;
//                auto const end1 = std::chrono::high_resolution_clock::now();
//                auto const delta_time = std::chrono::duration_cast<std::chrono::duration<double>> (end1 - start1);
//                std::cout << "store matrix cell cost " << delta_time.count() << "s\n";
                
                path.clear();
            }
                return true;
            }
            
            // for each current edge, calculate the travel time and record its node
            for (int num_streets = 0; num_streets < getNumIntersectionStreetSegment (curr.nodeID); num_streets++) {
                StreetSegmentIdx current_edge = getIntersectionStreetSegment (num_streets, curr.nodeID); // segment_id of current edge
                struct StreetSegmentInfo edge_info = getStreetSegmentInfo (current_edge); // info about current edge 

                IntersectionIdx toNodeID = 0;
                if (edge_info.oneWay == true) {
                    if (edge_info.from == curr.nodeID) {
                        toNodeID = edge_info.to; // node is at the from end of oneWay, look into it
                    } else {
                        continue; // node is at the to_end of segment, check the next direction
                    }
                } else {
                    if (edge_info.from == curr.nodeID) {
                        toNodeID = edge_info.to; // node is at from end, to_node is the next node
                    } else {
                        toNodeID = edge_info.from; // node is at to end, from_node is the next node
                    }
                }
                
                // If the next edge is able to reach, then identify its turn penalty
                double edge_travel_time = travelTimeM4(current_edge); // travel time of current edge
                if (curr.edgeID != NO_EDGE) {  // ignore the first start point
                    // get info about reaching 
                    StreetSegmentIdx reaching_edge = nodesM4[curr.nodeID].reachingEdge;
                    struct StreetSegmentInfo reaching_edge_info = getStreetSegmentInfo (reaching_edge);
                    // if current edge street id does not equal to reaching edge street id, then plus turn penalty
                    if (edge_info.streetID != reaching_edge_info.streetID) {
                        edge_travel_time += turn_penalty;
                    }
                }
                
                waveElem temp2;
                temp2.nodeID = toNodeID; temp2.edgeID = current_edge; temp2.travelTime = nodesM4[curr.nodeID].bestTime + edge_travel_time;
                wavefront.push(temp2);
                
//                auto const end1 = std::chrono::high_resolution_clock::now();
//                auto const delta_time_2 = std::chrono::duration_cast<std::chrono::duration<double>> (end1 - start1);
//                std::cout << "One while loop cost " << delta_time_2.count() << "s\n";
                //wavefront.push(waveElem (toNodeID, current_edge, nodesM4[curr.nodeID].bestTime + edge_travel_time));
            }
        }
    }
    return (false);
}

void convert_struct_to_vector (const std::vector<DeliveryInf>& deliveries, const std::vector<IntersectionIdx>& depots){
    
    // Store all the pickup and drop off and struct info into the matrix
    for (int num_of_deliveries = 0; num_of_deliveries < deliveries.size(); num_of_deliveries++) {
        struct intersection_info intersection_pickUp;
        intersection_pickUp.intersection_id = deliveries[num_of_deliveries].pickUp;
        intersection_pickUp.can_go = true;
        intersection_pickUp.visited = false;
        pickup_dropoff_depot.push_back(intersection_pickUp);
        
        struct intersection_info intersection_dropOff;
        intersection_dropOff.intersection_id = deliveries[num_of_deliveries].dropOff;
        intersection_dropOff.can_go = false;
        intersection_dropOff.visited = false;
        pickup_dropoff_depot.push_back(intersection_dropOff);
    }
    
    // store all the depots into the matrix
    for (int num_of_depots = 0; num_of_depots < depots.size(); num_of_depots++){
        struct intersection_info intersection_depot;
        intersection_depot.intersection_id = depots[num_of_depots];
        intersection_depot.can_go = true;
        intersection_depot.visited = false;
        pickup_dropoff_depot.push_back(intersection_depot);
    }
}
    
std::vector<StreetSegmentIdx> bfsTraceBackM4 (int destID, const std::vector<Node>& nodesM4) {
    std::list<StreetSegmentIdx> segment_path;
    int currNodeID = destID;
    StreetSegmentIdx prevEdge = nodesM4[currNodeID].reachingEdge;
    
    while (prevEdge != NO_EDGE) {
        segment_path.push_front (prevEdge);
        struct StreetSegmentInfo edge_info = getStreetSegmentInfo(prevEdge);
        
        // Update the current node 
        // If current node is at from end, then previous node is at to end of the edge, vice versa 
        if (currNodeID == edge_info.from) {
            currNodeID = edge_info.to;
        } else {
            currNodeID = edge_info.from;
        }
        prevEdge = nodesM4[currNodeID].reachingEdge; // Update prevEdge to store it for next loop
    }
    
    // convert list to vector and return the desired path 
    std::vector<StreetSegmentIdx> shortest_path (segment_path.begin(), segment_path.end());
    return shortest_path;
}

std::vector<CourierSubPath> travelingCourier (
                                                const float turn_penalty, 
                                                const std::vector<DeliveryInf>& deliveries,
                                                const std::vector<IntersectionIdx>& depots) {
    
    auto const start = std::chrono::high_resolution_clock::now();
    std::vector<CourierSubPath> return_vector;
    convert_struct_to_vector (deliveries, depots);
    load_unordered_multimap(deliveries, depots);
    matrix_travel_time.resize(pickup_dropoff_depot.size(), std::vector<matrix_element> (pickup_dropoff_depot.size())); // resize the matrix 
    
    #pragma omp parallel for 
    for (int num_rows = 0; num_rows < pickup_dropoff_depot.size(); num_rows++) {
        if (path_pre_compute (pickup_dropoff_depot[num_rows].intersection_id, turn_penalty, num_rows)) {
            continue;
        }
    }
    intersect_to_matrix_index.clear();
    
    auto const end = std::chrono::high_resolution_clock::now();
    auto const delta_time_1 = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
    std::cout << "compute matrix cost " << delta_time_1.count() << "s\n";
    
    
    auto const start_2 = std::chrono::high_resolution_clock::now();
    // Check if no legal route exist
    int numOfNonEmptyPath = 0;
    for(int row = 0; row < 2 * deliveries.size(); row++){
        for(int col = 0; col < 2 * deliveries.size(); col++){
            if(row != col && matrix_travel_time[row][col].from != matrix_travel_time[row][col].to &&
               matrix_travel_time[row][col].path_to_dest.size() == 0)
                return return_vector;
        }
    }
    
    for(int row = 0; row < 2 * deliveries.size() + depots.size(); row++){
        if(row < 2 * deliveries.size()){
            for(int col = 2 * deliveries.size(); col < 2 * deliveries.size() + depots.size(); col++){
                if(matrix_travel_time[row][col].path_to_dest.size() > 0)
                    numOfNonEmptyPath++;
            }
        }
        else{
            for(int col = 0; col < 2 * deliveries.size() + depots.size(); col++){
                if(matrix_travel_time[row][col].from != matrix_travel_time[row][col].to && 
                   matrix_travel_time[row][col].path_to_dest.size() > 0)
                    numOfNonEmptyPath++;
            }
        }
    }
    
    if(numOfNonEmptyPath < 2)
        return return_vector;
    
    // Find the first sub path
    std::vector<IntersectionIdx> BestPath;
    double shortestFirstSubPath = INFINITE;
    int startingDepotID = -1;
    int firstSubPathDest = -1;
    
    for(int row = 2 * deliveries.size(); row < 2 * deliveries.size() + depots.size(); row++){
        for(int col = 0; col < 2 * deliveries.size(); col++){
            if(pickup_dropoff_depot[col].can_go == true && 
               matrix_travel_time[row][col].travel_time < shortestFirstSubPath){
                startingDepotID = matrix_travel_time[row][col].from;
                firstSubPathDest = matrix_travel_time[row][col].to;
            }
        }
    }
    
    for(int intersection = 0; intersection < 2 * deliveries.size(); intersection++){
            
        // Found a match
        if(pickup_dropoff_depot[intersection].intersection_id == firstSubPathDest){

            if(intersection % 2 == 0){ // It is a pick up
                pickup_dropoff_depot[intersection].visited = true;
                pickup_dropoff_depot[intersection+1].can_go = true;
            }

            else if(intersection % 2 == 1 && pickup_dropoff_depot[intersection - 1].visited == true){
                pickup_dropoff_depot[intersection].visited = true;
            }
        }
    }
    
    BestPath.push_back(startingDepotID);
    BestPath.push_back(firstSubPathDest);
    
    //int count_temp = 0;
    while(1){
//        std::cout << count_temp << std::endl;
//        count_temp++;
        int subPathFrom = BestPath[BestPath.size() - 1];
        double shortestTime = INFINITE;
        int subPathTo = -1;
        bool found = false;
        
        // Find the row that has the from as src
        for(int row = 0; row < 2 * deliveries.size(); row++){
            
            // Found
            if(matrix_travel_time[row][0].from == subPathFrom){
                found = true;
                
                // Loop through col to find the closest available dest
                for(int col = 0; col < 2 * deliveries.size(); col++){
                    
                    if(row != col){ // Ignore Diagnol
                        
                        if(col % 2 == 0 && pickup_dropoff_depot[col].visited == false){ // Pick up that is not visited yet
                            
                            if(matrix_travel_time[row][col].travel_time < shortestTime){
                                shortestTime = matrix_travel_time[row][col].travel_time;
                                subPathTo = matrix_travel_time[row][col].to;
                            }
                        }
                        
                        else if(col % 2 == 1 && pickup_dropoff_depot[col].can_go == true &&
                                pickup_dropoff_depot[col].visited == false){ // Drop off that can go and is not visited yet
                            
                            if(matrix_travel_time[row][col].travel_time < shortestTime){
                                shortestTime = matrix_travel_time[row][col].travel_time;
                                subPathTo = matrix_travel_time[row][col].to;
                            }
                        }
                    }
                }
            }
            
            if(found)
                break;
        }
        
        // Update travel legality
        for(int intersection = 0; intersection < 2 * deliveries.size(); intersection++){
            
            // Found a match
            if(pickup_dropoff_depot[intersection].intersection_id == subPathTo){
                
                if(intersection % 2 == 0){ // It is a pick up
                    pickup_dropoff_depot[intersection].visited = true;
                    pickup_dropoff_depot[intersection+1].can_go = true;
                }
                
                else if(intersection % 2 == 1 && pickup_dropoff_depot[intersection - 1].visited == true){
                    pickup_dropoff_depot[intersection].visited = true;
                }
            }
        }
        
        // Push back to best path intersection vector if src and dest are different
        if(subPathTo != subPathFrom)
            BestPath.push_back(subPathTo);
        
        // Check if the route is finished
        bool pathFound = true;
        for(int intersection = 0; intersection < 2 * deliveries.size(); intersection++){
            if(pickup_dropoff_depot[intersection].visited == false){
                pathFound = false;
                break;
            }
        }
        
        if(pathFound)
            break;
    }
    
    // Find the last sub path (Depot)
    double shortestLastSubPath = INFINITE;
    int destDepotID = -1;
    int lastSubPathSrc = BestPath[BestPath.size() - 1];
    bool found = false;
    
    for(int row = 0; row < 2 * deliveries.size(); row++){
        if(matrix_travel_time[row][0].from == lastSubPathSrc){
            found = true;
            for(int col = 2 * deliveries.size(); col < 2 * deliveries.size() + depots.size(); col++){
                if(matrix_travel_time[row][col].travel_time < shortestLastSubPath){
                    shortestLastSubPath = matrix_travel_time[row][col].travel_time;
                    destDepotID = matrix_travel_time[row][col].to;
                }
            }
        }
        if(found)
            break;
    }
    
    BestPath.push_back(destDepotID);
    
    // Convert Best Path to vector of CourierSubPath
    for(int intersection = 0; intersection < BestPath.size() - 1; intersection++){
        CourierSubPath SubPath;
        SubPath.start_intersection = BestPath[intersection];
        SubPath.end_intersection = BestPath[intersection + 1];
        
        found = false;
        for(int row = 0; row < 2 * deliveries.size() + depots.size(); row++){
            if(matrix_travel_time[row][0].from == SubPath.start_intersection){
                found = true;
                for(int col = 0; col < 2 * deliveries.size() + depots.size(); col++){
                    if(matrix_travel_time[row][col].to == SubPath.end_intersection){
                        SubPath.subpath = matrix_travel_time[row][col].path_to_dest;
                        break;
                    }
                }
            }
            if(found)
                break;
        }
        
        return_vector.push_back(SubPath);
    }
    
    matrix_travel_time.clear();
    pickup_dropoff_depot.clear();
    
    auto const end2 = std::chrono::high_resolution_clock::now();
    auto const delta_time_2 = std::chrono::duration_cast<std::chrono::duration<double>> (end2 - start_2);
    std::cout << "The rest cost " << delta_time_2.count() << "s\n";
    
    return return_vector;
} 
