/*
 * Copyright 2022 University of Toronto
 *
 * Permission is hereby granted, to use this software and associated
 * documentation files (the "Software") in course work at the University
 * of Toronto, or for personal use. Other uses are prohibited, in
 * particular the distribution of the Software either publicly or to third
 * parties.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <iostream>
#include <string>
#include <algorithm>
#include <map>
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "math.h"

// A very big number used for spatial tests as the initial comparison value
#define MAX_DISTANCE 9999999999

/******************************* GLOBAL VARIABLES *******************************/
// They are loaded in loadMap and cleared in closeMap
std::vector<double> segment_travel_time;
std::vector<double> street_length;       // vector for street_length
std::vector<std::vector<StreetSegmentIdx>> intersection_street_segments;
std::vector<std::vector<std::string>> intersection_street_names;
std::vector<std::vector<IntersectionIdx>> intersections_of_streets;
std::unordered_map<OSMID, LatLon> latLon_of_OSMNodes;
std::multimap<std::string, StreetIdx > streetNames;

// Functions that load the Global Variables
void load_segment_travel_time();
void load_street_length();
void load_intersection_street_segments();
void load_intersection_street_names();
void load_intersections_of_streets();
void load_latLon_of_OSMNodes();
void loadStreetNames();
void insertDivisionStreetNames();
void preParse(std::string &prefix);
auto findStartLoc(std::string prefix);
auto findEndLoc(std::string prefix);

/******************************* HELPER FUNCTIONS *******************************/
// They help load map related data
// They are called in loadMap and makes load map shorter


// Go through each segment and see which street it belongs to,
//and then Pre-calculate the length of each street to save time in function findStreetLength
void load_street_length() {
    street_length.resize(getNumStreets());
    
    for (StreetSegmentIdx streetSegId = 0; streetSegId < getNumStreetSegments(); streetSegId++) {
    
    StreetSegmentInfo streetSegment = getStreetSegmentInfo(streetSegId);
    
    // Use the findStreetSegmentLength function to find the length of each segment
    double streetSegLength = findStreetSegmentLength(streetSegId);
    
    // Go through each segment and add up all the segment length that belong to same street_id.
    // Store the length of the street according to its street_id into vector, can be accessed by index 
    street_length[streetSegment.streetID] += streetSegLength;
    }
}

// Go through each street segments, 
// and pre-load the travel time of each segment and store it into the vector of segment_travel_time to save the time in travel time function
void load_segment_travel_time() {
    segment_travel_time.resize(getNumStreetSegments());
    
    for (StreetSegmentIdx streetSegId = 0; streetSegId < getNumStreetSegments(); streetSegId++) {
        StreetSegmentInfo streetSegment = getStreetSegmentInfo(streetSegId);
        
        // Find the speedLimit of the each street segment and implement calculation
        double segTravelTime = findStreetSegmentLength(streetSegId) / streetSegment.speedLimit;
        
        // Store the travel time of each segment according to its segment_id into the vector
        segment_travel_time[streetSegId] = segTravelTime;
    }
}

//A function that inserts "dividers" in the sorted multimap in alphabetic order. Dividers are strings from A-Z and AA-ZZ
void insertDivisionStreetNames(){
	const char letterStart = 'A';
	char firstChar = letterStart;
	char secondChar = letterStart;
	std::string strtemp;

	//insert singel letter divider
	for (int i = 0; i < 26; i++){
		strtemp.push_back(firstChar);
		firstChar = firstChar + 1;
		streetNames.emplace(strtemp, 0);
		strtemp.erase();
	}

	//reset firstChar
	firstChar = letterStart;

	//insert double letter divisor
	for (int i = 0; i < 26; i++){
		strtemp.push_back(firstChar);
		firstChar = firstChar + 1;
		for (int counter = 0; counter < 26; counter++){
			strtemp.push_back(secondChar);
			secondChar = secondChar + 1;
			streetNames.emplace(strtemp, 0);
			strtemp.pop_back();
		}
		//reset strtemp and second letter
		strtemp.erase();
		secondChar = letterStart;
	}
}

//A function that takes a string by reference and convert all of its letters to upper case
void preParse(std::string &prefix){
	//ascii difference between upper and lower case
    const int caseDiff = 32;

	for (int counter = 0; prefix[counter] != '\0'; counter++){
        if ('a' <= prefix[counter] && 'z' >= prefix[counter]){
			prefix[counter] -= caseDiff;	
		}
	}
}

//A function that takes a string and uses its first (or first two characters) to locate the starting divider position in streetNames multi-map, 
//and returns a iterator pointing at one after the divider 
auto findStartLoc(std::string prefix){
	char firstChar;
	char secondChar;
	std::string strtemp;

	//when prefix is one character long
	firstChar = prefix[0];
	if (!isalpha(firstChar)){
		auto itStart = streetNames.begin();
		return itStart;
	}
	strtemp.push_back(firstChar);

	//when prefix is two or more character long
	if (prefix.size() >= 2){
		secondChar = prefix[1];
		strtemp.push_back(secondChar);
		auto itStart = streetNames.find(strtemp);
		itStart++;
		return itStart;
	}
    
	auto itStart = streetNames.find(strtemp);
	itStart++;
	return itStart;
}

//A funciton that takes a string and uses its first (or first two characters) to locate the ending divider position in streetNames multimap, 
//and returns a iterator poiting at the divider
auto findEndLoc(std::string prefix){
	char firstChar;
	char secondChar;
	std::string strtemp;

	//when prefix is one character long
	firstChar = prefix[0];
	if (!isalpha(firstChar)){
		auto itStart = streetNames.find("A");
		return itStart;
	}
	strtemp.push_back(firstChar);

	//when prefix is two or more character long
	if (prefix.size() >= 2){
		secondChar = prefix[1];
		secondChar += 1;
		strtemp.push_back(secondChar);
		auto itStart = streetNames.find(strtemp);
		return itStart;
	}

	strtemp.erase();
	firstChar += 1;
	strtemp.push_back(firstChar);
	auto itStart = streetNames.find(strtemp);
	return itStart;
} 

void loadStreetNames(){
    std::string temp;
	for (int counter = 0; counter < getNumStreets(); counter++){
        temp = getStreetName(counter);
        temp.erase(std::remove(temp.begin(), temp.end(), ' '), temp.end());

        preParse(temp);

    	streetNames.emplace(temp, counter);
	}
	insertDivisionStreetNames();
}

// Get a vector of vectors of the street segment indices of each intersection
// The vectors of street segment indices of an intersection can be accessed with its intersection index
void load_intersection_street_segments(){
    
    intersection_street_segments.resize(getNumIntersections());
  	 
    for(int intersection = 0; intersection < getNumIntersections(); intersection++){
  		 
   	 for(int streetSegNum = 0; streetSegNum < getNumIntersectionStreetSegment(intersection); streetSegNum++){
      		 
   		 int ss_id = getIntersectionStreetSegment(streetSegNum, intersection);
   		 intersection_street_segments[intersection].push_back(ss_id);
   	 }
    }
}


// Get a vector of vectors of the street names at each intersection
// The vectors of street names of an intersection can be accessed with its intersection index
void load_intersection_street_names(){

	intersection_street_names.resize(getNumIntersections());

	for(int intersection = 0; intersection < getNumIntersections(); intersection++){

    	for(int streetSeg = 0; streetSeg < intersection_street_segments[intersection].size(); streetSeg++){

        	StreetSegmentInfo SSInfo = getStreetSegmentInfo(intersection_street_segments[intersection][streetSeg]);
        	intersection_street_names[intersection].push_back(getStreetName(SSInfo.streetID));
    	}
	}
}

// Get a vector of vectors of the intersection indices for each street
// The vectors of intersection indices of a street can be accessed with its street index
void load_intersections_of_streets(){
    
	intersections_of_streets.resize(getNumStreets());

	// Add from and to intersections of all street segments to their corresponding street vector
	for(int streetSegIdx = 0; streetSegIdx < getNumStreetSegments(); streetSegIdx++){
    	StreetSegmentInfo streetSegment = getStreetSegmentInfo(streetSegIdx);
    	intersections_of_streets[streetSegment.streetID].push_back(streetSegment.from);
    	intersections_of_streets[streetSegment.streetID].push_back(streetSegment.to);
	}

	// Remove duplicates by sorting street vector and using unique and erase functions
	for(int street = 0; street < getNumStreets(); street++){
    	std::sort(intersections_of_streets[street].begin(), intersections_of_streets[street].end());
    	auto lastNotRemoved = std::unique(intersections_of_streets[street].begin(), intersections_of_streets[street].end());
    	intersections_of_streets[street].erase(lastNotRemoved, intersections_of_streets[street].end());
	}
}

// Get an unordered map of the LatLon of all OSM Nodes
// The LatLon of each OSM Nodes can be accessed with its OSM id
void load_latLon_of_OSMNodes(){
	for(int node = 0; node < getNumberOfNodes(); node++){
    	OSMID id = getNodeByIndex(node) -> id();
    	LatLon nodeCoords = getNodeCoords(getNodeByIndex(node));
    	latLon_of_OSMNodes.insert(std::make_pair(id, nodeCoords));
	}
}

// loadMap will be called with the name of the file that stores the "layer-2"
// map data accessed through StreetsDatabaseAPI: the street and intersection
// data that is higher-level than the raw OSM data).
// This file name will always end in ".streets.bin" and you
// can call loadStreetsDatabaseBIN with this filename to initialize the
// layer 2 (StreetsDatabase) API.
// If you need data from the lower level, layer 1, API that provides raw OSM
// data (nodes, ways, etc.) you will also need to initialize the layer 1
// OSMDatabaseAPI by calling loadOSMDatabaseBIN. That function needs the
// name of the ".osm.bin" file that matches your map -- just change
// ".streets" to ".osm" in the map_streets_database_filename to get the proper
// name.
bool loadMap(std::string map_streets_database_filename) {
    
	// Remove the street database file extension and concatenate OSM database file extension to load OSM database
	std::string osm_file_path_suffix = ".osm.bin";
	int dot_position = map_streets_database_filename.find(".");
	std::string filename_without_extension = map_streets_database_filename.substr(0, dot_position);
	std::string osm_file_path = filename_without_extension + osm_file_path_suffix;

	// Indicates whether the two databases are loaded successfully
	bool StreetDatabase_load_successful = loadStreetsDatabaseBIN(map_streets_database_filename);
	bool OSMDatabase_load_successful = loadOSMDatabaseBIN(osm_file_path);

	std::cout << "loadMap: " << map_streets_database_filename << std::endl;

	// Load map data if the map is successfully loaded
	if(StreetDatabase_load_successful && OSMDatabase_load_successful){
    	load_intersection_street_segments();
    	load_intersection_street_names();
    	load_intersections_of_streets();
    	load_latLon_of_OSMNodes();
    	loadStreetNames();
        load_street_length();
        load_segment_travel_time();
	}

	return (StreetDatabase_load_successful && OSMDatabase_load_successful);
}

// Function that unload the map
void closeMap() {

	//Clean-up your map related data structures here
	closeStreetDatabase();
	closeOSMDatabase();

	// Clear Global Variables
	intersection_street_segments.clear();
	intersections_of_streets.clear();
	intersection_street_names.clear();
	latLon_of_OSMNodes.clear();
	streetNames.clear();
        street_length.clear();
        segment_travel_time.clear();

}

// Function that uses an intersection id to return a vector of its street segment indices
std::vector<StreetSegmentIdx> findStreetSegmentsOfIntersection(IntersectionIdx intersection_id){
	return intersection_street_segments[intersection_id];
}

// Function that uses an intersection id to return a vector of its street names
std::vector<std::string> findStreetNamesOfIntersection(IntersectionIdx intersection_id){
	return intersection_street_names[intersection_id];
}

// Function that uses two intersection indices to return a boolean indicating whether they are directly connected or not
bool intersectionsAreDirectlyConnected(std::pair<IntersectionIdx, IntersectionIdx> intersection_ids){
    
	// Return Value
	bool areConnected = false;

	// Access the vectors of street segment indices of the two intersections
	std::vector<StreetSegmentIdx> SSIDX_of_1st_Intersection = intersection_street_segments[intersection_ids.first];
	std::vector<StreetSegmentIdx> SSIDX_of_2nd_Intersection = intersection_street_segments[intersection_ids.second];

	// Use a nested for loop to check if two vectors have a identical street segment index
	for(int SS_Intersection1 = 0; SS_Intersection1 < SSIDX_of_1st_Intersection.size(); SS_Intersection1++){
	for(int SS_Intersection2 = 0; SS_Intersection2 < SSIDX_of_2nd_Intersection.size(); SS_Intersection2++){

        	// Having the same street segment means the two intersections are directly connected
        	if(SSIDX_of_1st_Intersection[SS_Intersection1] == SSIDX_of_2nd_Intersection[SS_Intersection2]){
        	areConnected = true;
        	break;
        	}
	}
	if(areConnected)
        	break;
	}

	return areConnected;
}

// Function that uses a LatLon of a position to return the LatLon of the closest intersection to the given position
IntersectionIdx findClosestIntersection(LatLon my_position){
	// Initialize return value and set a high initial value in closestIntersection for distance comparison
	int closestIntersectionIdx = 0;
	double closestDistance = MAX_DISTANCE;
    
	// Compare distance to all intersections
	for(int intersection = 0; intersection < getNumIntersections(); intersection++){
    	LatLon intersectionPosition = getIntersectionPosition(intersection);
    	double distance = findDistanceBetweenTwoPoints(std::make_pair(my_position, intersectionPosition));
   	 
    	// Updates the closest intersection and its index
    	if(distance < closestDistance){
        	closestDistance = distance;
        	closestIntersectionIdx = intersection;
    	}
	}
	return closestIntersectionIdx;
}

// Function that uses a street id to return a vector of all its intersection indices
std::vector<IntersectionIdx> findIntersectionsOfStreet(StreetIdx street_id){
	return intersections_of_streets[street_id];
}

// Function that uses two street indices to return a vector of their common intersection indices
std::vector<IntersectionIdx> findIntersectionsOfTwoStreets(std::pair<StreetIdx, StreetIdx> street_ids){
    
	// Obtain Intersections of two streets as two vectors, create a vector that stores their common intersections
	std::vector<int> IntersectionsOfFirstStreet = findIntersectionsOfStreet(street_ids.first);
	std::vector<int> IntersectionsOfSecondStreet = findIntersectionsOfStreet(street_ids.second);
	std::vector<int> commonIntersectionsOfTwoStreets(IntersectionsOfFirstStreet.size() + IntersectionsOfSecondStreet.size());
	std::vector<int>::iterator startOfCommonIntersections, endOfCommonIntersections;
    
	startOfCommonIntersections = commonIntersectionsOfTwoStreets.begin();
    
	endOfCommonIntersections = std::set_intersection(IntersectionsOfFirstStreet.begin(), IntersectionsOfFirstStreet.end(),	// Start and End Iterators of Vector 1
                                                 	IntersectionsOfSecondStreet.begin(), IntersectionsOfSecondStreet.end(),  // Start and End Iterators of Vector 2
                                                 	startOfCommonIntersections);                                         	// Start Iterator of storing common elements
                                                                                                                          	// Return End Iterator of storing common elements
	// Resize the return vector with the Start and End Iterators of storing comment intersections
	commonIntersectionsOfTwoStreets.resize(endOfCommonIntersections - startOfCommonIntersections);
	return commonIntersectionsOfTwoStreets;
}

// Function that uses a LatLon of a position and a string of POI type to return the closest POI of the given type to the given position
POIIdx findClosestPOI(LatLon my_position, std::string POItype){
	// Initialize return value and set a high initial value in closestIntersection for distance comparison
	int ClosestPOIIndex = 0;
	double closestDistance = MAX_DISTANCE;
    
	// Compare distance to all POIs of the right type
	for(int POI = 0; POI < getNumPointsOfInterest(); POI++){
    	if(getPOIType(POI) != POItype)
        	continue;
    	LatLon POIPosition = getPOIPosition(POI);
    	double distance = findDistanceBetweenTwoPoints(std::make_pair(my_position, POIPosition));
   	 
    	// Updates the closest POI and its index
    	if(distance < closestDistance){
        	closestDistance = distance;
        	ClosestPOIIndex = POI;
    	}
	}
    
	return ClosestPOIIndex;
}

// Function that uses an OSM ID to return the LatLon of the OSM Node with the given ID
LatLon findLatLonOfOSMNode (OSMID OSMid){
	return latLon_of_OSMNodes.find(OSMid) -> second;
}

std::vector<StreetIdx> findStreetIdsFromPartialStreetName(std::string street_prefix){
	std::vector <StreetIdx> returnVector;

    street_prefix.erase(std::remove(street_prefix.begin(), street_prefix.end(), ' '), street_prefix.end());
	preParse(street_prefix);

	if (street_prefix == "") return returnVector;

	auto startLoc = findStartLoc(street_prefix);
	auto endLoc = findEndLoc(street_prefix);

	for (auto it = startLoc; it != endLoc; it++){
    	if ((it ->first).compare(0, street_prefix.size(), street_prefix) == 0 && street_prefix.size() <= (it -> first).size()){
        	returnVector.push_back(it -> second);
    	}
	}

	return returnVector;
}

// Function that calculates the distance between to points 
double findDistanceBetweenTwoPoints(std::pair<LatLon, LatLon> points) {
    
    // Get the information of two points that are used to calculate
    LatLon firstPoint = points.first;
    LatLon secondPoint = points.second;
    
    // Use the formula provided in the m1 handout 
    double x1_coor = kEarthRadiusInMeters * firstPoint.longitude()* kDegreeToRadian * cos (kDegreeToRadian * (firstPoint.latitude() + secondPoint.latitude()) / 2);
    double x2_coor = kEarthRadiusInMeters * secondPoint.longitude() * kDegreeToRadian * cos (kDegreeToRadian * (firstPoint.latitude() + secondPoint.latitude()) / 2);

    double y1_coor = kEarthRadiusInMeters * firstPoint.latitude() * kDegreeToRadian;
    double y2_coor = kEarthRadiusInMeters * secondPoint.latitude()  * kDegreeToRadian;
    
    double distance = sqrt (pow((y2_coor - y1_coor), 2) + pow((x2_coor - x1_coor), 2));

    return distance;
}

// Function that finds the length of each street segment with given streetSegmentId
double findStreetSegmentLength(StreetSegmentIdx street_segment_id){
    StreetSegmentInfo streetSegmentInfo = getStreetSegmentInfo(street_segment_id);

    LatLon from = getIntersectionPosition(streetSegmentInfo.from);
    LatLon to = getIntersectionPosition(streetSegmentInfo.to);  
    // Get position info of the start and the end point of a street segment
    
    if (streetSegmentInfo.numCurvePoints == 0) {
        std::pair <LatLon,LatLon> length;
        length = std::make_pair(from, to);
        return findDistanceBetweenTwoPoints(length); 
        // The case that has no curve points, length is directly from begin to end
        
    } else if (streetSegmentInfo.numCurvePoints == 1){
        LatLon ssCurvePoints = getStreetSegmentCurvePoint((streetSegmentInfo.numCurvePoints - 1), street_segment_id);
        
        std::pair <LatLon,LatLon> firstCurve;
        std::pair <LatLon,LatLon> lastCurve;
        
        firstCurve = std::make_pair(from, ssCurvePoints);
        lastCurve = std::make_pair(ssCurvePoints, to);
        return findDistanceBetweenTwoPoints(firstCurve) + findDistanceBetweenTwoPoints(lastCurve);
        // Have one curve point
        
    } else {
        double midLength = 0, totalLength = 0, firstAndLastSeg = 0;
        for (int i = 0; i < (streetSegmentInfo.numCurvePoints - 1); i++) {
            // Get the position of current curve point and next curve point
            LatLon ssCurvePoints = getStreetSegmentCurvePoint(i, street_segment_id);
            LatLon ssCurvePoints_Next = getStreetSegmentCurvePoint((i + 1), street_segment_id);
            
            // Make pair of two consecutive curve points
            std::pair <LatLon,LatLon> curveLength;
            curveLength = std::make_pair(ssCurvePoints, ssCurvePoints_Next);
            
            // Add middle length each time 
            midLength += findDistanceBetweenTwoPoints(curveLength);  // length without first and last segment
        }
           
        // Get positions of first and last curve points
        LatLon firstPoint = getStreetSegmentCurvePoint(0, street_segment_id); 
        LatLon lastPoint = getStreetSegmentCurvePoint((streetSegmentInfo.numCurvePoints - 1), street_segment_id); 
        
        // Create pairs of LatLons for first and last segments
        std::pair <LatLon,LatLon> firstCurve;
        std::pair <LatLon,LatLon> lastCurve;
        
        firstCurve = std::make_pair(from, firstPoint);
        lastCurve = std::make_pair(lastPoint, to);
        
        // Add middle length, first segment, and last segment together
        firstAndLastSeg = findDistanceBetweenTwoPoints(firstCurve) + findDistanceBetweenTwoPoints(lastCurve);
        totalLength = firstAndLastSeg + midLength; 
        return totalLength;
    }
}
 
// Access by index to get the travel time for that street_segment_id
double findStreetSegmentTravelTime(StreetSegmentIdx street_segment_id) {
    return segment_travel_time[street_segment_id];
}

// Access by index to get the length of the street for that street_id
double findStreetLength(StreetIdx street_id) {
    return street_length[street_id];
}   

double findFeatureArea(FeatureIdx feature_id) {
    std::vector <LatLon> pointsInfo;
    
    // A vector that contains all the feature points with longitude and latitude
    int numFeathurePoints = getNumFeaturePoints(feature_id);
    for (int i = 0; i < numFeathurePoints; i++) {
        LatLon featurePoint = getFeaturePoint (i, feature_id);
        pointsInfo.push_back (featurePoint);
    }
    
    // If the first point (point 0)and last point (point featurePoint -1 ) are NOT the same, 
    // then it is not a polygon, return 0
    LatLon beginPoint = getFeaturePoint(0, feature_id);
    LatLon endPoint = getFeaturePoint (numFeathurePoints - 1, feature_id);
    
    if ((beginPoint.latitude() != endPoint.latitude()) || (beginPoint.longitude() != endPoint.longitude())) {
        return 0;
    } 
    
    double area = 0;
    for (int i = 0; i < pointsInfo.size() - 1; i++) {
        
        // Get the information of two points  
        LatLon firstPoint = pointsInfo[i];
        LatLon secondPoint = pointsInfo[i + 1];
        
        // Use the formula taught in ECE297 lecture: The sum of (y2 - y1) * ((x2 + x1) / 2) for each feature point, range from 0 - N-1
        // Convert from latitude and longitude to x and y
        double x1_coor = (kEarthRadiusInMeters * firstPoint.longitude()* kDegreeToRadian * cos (kDegreeToRadian * (firstPoint.latitude() + secondPoint.latitude()) / 2));
        double x2_coor = (kEarthRadiusInMeters * secondPoint.longitude()* kDegreeToRadian * cos (kDegreeToRadian * (firstPoint.latitude() + secondPoint.latitude()) / 2));
        
        double y1_coor = (kEarthRadiusInMeters * firstPoint.latitude() * kDegreeToRadian);
        double y2_coor = (kEarthRadiusInMeters * secondPoint.latitude() * kDegreeToRadian);
        
        // Go through each point to calculate the total area
        area += (y2_coor - y1_coor) * ((x1_coor + x2_coor) / 2);
    }
    pointsInfo.clear();
    
    return abs(area);
}
