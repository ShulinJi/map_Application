/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cppFiles/file.cc to edit this template
 */
#include "m3.h"
#include <utility>
#include <vector>
#include <list>
#include <algorithm>
#include <queue>
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "m2.h"
#include "m1.h"

#define NO_EDGE -1
#define INFINITE 9999999999
#define MAX_SPEED 33.333333
 
struct waveElem {
    IntersectionIdx nodeID;  // current node 
    StreetSegmentIdx edgeID; // edge to reach current node
    double travelTime;       // Total travel time to reach the node
    double estimatingTime;
    
    waveElem (int node, int edge, float travel_time, float estimating_time) {
        nodeID = node; edgeID = edge; travelTime = travel_time; estimatingTime = estimating_time;
    }
    
    bool operator< (const waveElem & rhs) const {
        if (estimatingTime > rhs.estimatingTime) {
            return true;
        } else {
            return false;
        } 
    }
    
};

class Node {
public:
    //StreetSegmentIdx outEdge;
    StreetSegmentIdx reachingEdge; // edge used to reach current node
    double bestTime = INFINITE; // shortest time to get this node 
};

std::vector<Node> nodes;
double travelTime (StreetSegmentIdx outEdge);
bool bfsPath (int srcID, int destID, const double turn_penalty);
std::vector<StreetSegmentIdx> bfsTraceBack (int destID);
double findEstimatedTime (IntersectionIdx srcID, IntersectionIdx destID);

double travelTime (StreetSegmentIdx outEdge) {
    return findStreetSegmentTravelTime (outEdge);
}

double findEstimatedTime (IntersectionIdx srcID, IntersectionIdx destID) {
    
    LatLon curr_inter = getIntersectionPosition (srcID);
    LatLon dest_inter = getIntersectionPosition (destID);
    auto inter_pair = std::make_pair (curr_inter, dest_inter);
    double estimated_dis = findDistanceBetweenTwoPoints(inter_pair);
    double estimated_time = estimated_dis / MAX_SPEED;
    return estimated_time;
    
}

bool bfsPath (int srcID, int destID, const double turn_penalty) {
    std::priority_queue<waveElem> wavefront;
    wavefront.push (waveElem (srcID, NO_EDGE, 0, findEstimatedTime(srcID, destID)));
    
    while (wavefront.size() != 0) {
        waveElem curr = wavefront.top();
        wavefront.pop();
        
        if (curr.travelTime < nodes[curr.nodeID].bestTime) {
            nodes[curr.nodeID].reachingEdge = curr.edgeID;
            nodes[curr.nodeID].bestTime = curr.travelTime;
            
            if (curr.nodeID == destID) {
                return (true);
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
                double edge_travel_time = travelTime(current_edge); // travel time of current edge
                if (curr.edgeID != NO_EDGE) {  // ignore the first start point
                    // get info about reaching 
                    StreetSegmentIdx reaching_edge = nodes[curr.nodeID].reachingEdge;
                    struct StreetSegmentInfo reaching_edge_info = getStreetSegmentInfo (reaching_edge);
                    // if current edge street id does not equal to reaching edge street id, then plus turn penalty
                    if (edge_info.streetID != reaching_edge_info.streetID) {
                        edge_travel_time += turn_penalty;
                    }
                }
                
                // find estimating time from current point to destination by distance / 33.3333m/s (120km/h)
                // to avoid any overestimation 
                double estimated_time = findEstimatedTime (toNodeID, destID);
                
                nodes[curr.nodeID].bestTime = curr.travelTime;
                wavefront.push(waveElem (toNodeID, current_edge, nodes[curr.nodeID].bestTime + edge_travel_time, 
                                        nodes[curr.nodeID].bestTime + edge_travel_time + estimated_time));
            }
        }
    }
    return (false);
}

std::vector<StreetSegmentIdx> bfsTraceBack (int destID) {
    std::list<StreetSegmentIdx> segment_path;
    int currNodeID = destID;
    StreetSegmentIdx prevEdge = nodes[currNodeID].reachingEdge;
    
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
        prevEdge = nodes[currNodeID].reachingEdge; // Update prevEdge to store it for next loop
    }
    
    // convert list to vector and return the desired path 
    std::vector<StreetSegmentIdx> shortest_path (segment_path.begin(), segment_path.end());
    return shortest_path;
}

double computePathTravelTime (const double turn_penalty, const std::vector<StreetSegmentIdx>& path) {
    double travel_time = 0;
    if (path.size() == 0) {
        return 0;
    } else {
        for (int num_of_edge = 0; num_of_edge < path.size(); num_of_edge++) {
            int curr_edge = path[num_of_edge]; // segment id of current edge
            struct StreetSegmentInfo segment_info = getStreetSegmentInfo(curr_edge);
            travel_time += travelTime (curr_edge); 
            
            // To see if there is an turn 
            // and ignore for last edge
            if (num_of_edge < path.size() - 1) {
                int next_edge = path[num_of_edge + 1]; // segment id of next edge
                struct StreetSegmentInfo next_segment_info = getStreetSegmentInfo(next_edge); // get info of next segment

                // If the current edge has the same streetID as next edge, then it does not have a turn, vice versa
                if (next_segment_info.streetID == segment_info.streetID) {
                    continue;
                } else {
                    travel_time += turn_penalty;
                }
            }
        }
    }
    return travel_time;
}

std::vector<StreetSegmentIdx> findPathBetweenIntersections (
                        const double turn_penalty, 
                        const std::pair <IntersectionIdx, IntersectionIdx> intersection_ids) {
    
    nodes.resize(getNumIntersections()); // resize the calculation for next expansion 
    std::vector<StreetSegmentIdx> bestPath;
    
    // If there is a path, return the path 
    if (bfsPath(intersection_ids.first, intersection_ids.second, turn_penalty)) {
        bestPath = bfsTraceBack (intersection_ids.second);
        //std::cout << "found a path" << "\n";
    }
    nodes.clear();
    return bestPath;
}