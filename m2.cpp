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

#include "MoreFindFromPartialName.h"
#include "m2.h"
#include "m1.h"
#include "OSMDatabaseAPI.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"
#include "math.h"
#include <utility>
#include <sstream>
#include <string>
#include <algorithm>
#include <chrono>

int level_of_detail; // level of the details that are shown on the screen
double max_area; // maximum visible area of the city

std::vector<std::string> POI_TYPES = {"bar", "biergarten", "cafe", "fast_food", "food_court", "ice_cream", "pub", "restaurant", 
                                      "college", "driving_school", "kindergarten", "language_school", "library", "toy_library",
                                      "music_school", "school", "university", "bicycle_parking", "bicycle_repair_station",
                                      "bicycle_rental", "boat_rental", "boat_sharing", "bus_station", "car_rental", "car_sharing",
                                      "car_wash", "vehicle_inspection", "charging_station", "ferry_terminal", "fuel", "grit_bin",
                                      "motorcycle_parking", "parking", "parking_entrance", "parking_space", "taxi", "atm", "bank",
                                      "bureau_de_change", "baby_hatch", "clinic", "dentist", "doctors", "hospital", "nursing_home",
                                      "pharmacy", "social_facility", "veterinary", "arts_centre", "brothel", "casino", "cinema",
                                      "community_centre", "conference_centre", "events_venue", "fountain", "gambling", "love_hotel", 
                                      "nightclub", "planetarium", "public_bookcase", "social_centre", "stripclub", "studio", 
                                      "swingerclub", "theatre", "courthouse", "fire_station", "police", "post_box", "post_depot", 
                                      "post_office", "prison", "ranger_station", "townhall", "bbq", "bench", "dog_toilet", 
                                      "drinking_water", "give_box", "parcel_locker", "shelter", "shower", "telephone", "toilets", 
                                      "water_point", "watering_place", "sanitary_dump_station", "recycling", "waste_basket", 
                                      "waste_disposal", "waste_transfer_station", "animal_boarding", "animal_breeding", 
                                      "animal_shelter", "baking_oven", "childcare", "clock", "crematorium", "dive_centre", 
                                      "funeral_hall", "grave_yard", "hunting_stand", "internet_cafe", "kitchen", "kneipp_water_cure", 
                                      "lounger", "marketplace", "monastery", "photo_booth", "place_of_mourning", "place_of_worship", 
                                      "public_bath", "refugee_site", "vending_machine"};

std::vector<std::string> FONTS = {"Noto Sans", "Noto Sans CJK SC", "Noto Sans CJK JP"};

std::vector<std::vector<ezgl::point2d>> xy_lowest_level_way_point;
std::vector<std::vector<ezgl::point2d>> xy_special_way_point;

double max_lat;
double min_lat;
double max_lon;
double min_lon;

float x_min;
float x_max;
float y_min;
float y_max;

int language = 0;

bool Display_All_POIs = false;
bool Display_All_Subways = false;

double Cos_LatLon_to_XY;
std::string Map_Name = "toronto_canada";

GtkEntry *text_entry;
GObject *search_entry;
GObject *change_map;

struct xy_coordinate {
    ezgl::point2d from;
    ezgl::point2d to;
};

struct intersection_data {
    ezgl::point2d xy_loc;
    std::string name;
    bool highlight;
};

struct POI_data {
    ezgl::point2d xy_loc;
    std::string type;
    std::string name;
    bool highlight;
};

std::vector<xy_coordinate> xy_coordinate_version; // Vector of structs contains all the x and y coordinate of from and to in each street segment
std::vector<intersection_data> intersections; // Vector that stores the intersection data
std::vector<POI_data> POIs; // // Vector that stores the POI data
struct segment_name_data {
    ezgl::point2d from;
    ezgl::point2d to;
    ezgl::point2d center;
    std::string streetName;
    double x_bound, y_bound;
    double rotation;
    int curved_pointes;
    int segID;
    bool oneWay;
    bool curved;
    bool draw;
};

std::vector<std::vector<StreetSegmentIdx>> connectedStreetSegments; //double vector holds segments of different streets, orderd

std::vector<StreetSegmentIdx> osm_highway_motorway;
std::vector<StreetSegmentIdx> osm_highway_trunk;
std::vector<StreetSegmentIdx> osm_highway_primary;
std::vector<StreetSegmentIdx> osm_highway_secondary;
std::vector<StreetSegmentIdx> osm_highway_tertiary;
std::vector<StreetSegmentIdx> lowest_level_way;     // data structure for different kinds of road
std::vector<StreetSegmentIdx> special_way;
std::unordered_map<OSMID, const OSMWay*> id_to_OSMWay; // data structure to access the OSMWay* by its OSMID
std::vector<FeatureIdx> lake_feature;
std::vector<FeatureIdx> park_feature;
std::vector<FeatureIdx> beach_feature;
std::vector<FeatureIdx> river_feature;
std::vector<FeatureIdx> island_feature;
std::vector<FeatureIdx> building_feature;
std::vector<FeatureIdx> building_ordered;
std::vector<FeatureIdx> greenspace_feature;
std::vector<FeatureIdx> golfcourse_feature;
std::vector<FeatureIdx> stream_feature;
std::vector<FeatureIdx> glacier_feature;
std::vector<std::vector<ezgl::point2d>> xy_coor_subway;
std::multimap <double, FeatureIdx> building_area;
std::vector<segment_name_data> segment_names;

std::vector<std::vector<ezgl::point2d>> xy_all_feature_point;
std::vector<std::vector<ezgl::point2d>> xy_all_segment_point;
void pre_load_world_data ();


void sort_building_by_area (std::vector<FeatureIdx> building);
void coordinate_transformation (); // transform all the from and to Latlon to xy coordinate 
ezgl::point2d make_point_xy (LatLon curve_point); // convert a point to x-y coordinate

void find_map_bound_and_load_intersections(); // find the map bound and load all the intersection data
void update_level_of_detail (ezgl::renderer *g); // update the number of level_of_detail to control visible stuff
void load_subway();
void load_main_minor_road(); // load street id for each kind of road
void load_feature_points(); // load feature_id for each kind of feature
void draw_main_canvas (ezgl::renderer *g); // function prototype for drawing canvas
void draw_all_street_segments (ezgl::renderer *g); // function to draw all the street segments
void draw_main_minor_road(ezgl::renderer *g); // draw all the roads with some of them highlighted
void draw_feature_area(ezgl::renderer *g); // draw all the features in order
void draw_different_feature_area(ezgl::renderer *g, std::vector<FeatureIdx> feature_vector, int red, int green, int blue, int line_width); // helper function to draw one kind of feature
void draw_building_first_layer_area(ezgl::renderer *g, std::vector<FeatureIdx> building_order, int red, int green, int blue, int line_width, int size);
void draw_one_street_segment (ezgl::renderer *g, StreetSegmentIdx segment_id, int red, int green, int blue, int opaque, int line_width); // helper function to draw one kind of street segments
void draw_subway (ezgl::renderer *g);
void draw_subway_lines(GtkWidget* /*widget*/, ezgl::application* app);

void load_intersection();
void load_all_street_segments();

double find_distance_between_two_point(double x0, double y0, double x1, double y1);
bool rotation_adjust(double x0, double y0, double x1, double y1, double &rotation);
void find_and_set_longest_straight_seg(int segID);
void set_x_bond_and_center_and_rotation(int segID);
void load_all_segments_names();
void draw_one_segment_name(ezgl::renderer *g, StreetSegmentIdx segID);
void set_secondaryName_y_bound();
void set_tertiaryName_y_bound();
void set_smallName_y_bound();


void load_POIs();
void draw_highlighted_POIs(ezgl::renderer *g);
void draw_highlighted_intersections(ezgl::renderer *g);// draw the nearest intersection when mouse clicked
void draw_all_POIs(ezgl::renderer *g);

double x_from_lon(float lon, double cos);
double y_from_lat(float lat);
double lon_from_x(float x, double cos);
double lat_from_y(float y);
void centre_map_at_a_point(double point_x, double point_y, ezgl::application* app);
void clear_globals();

void act_on_mouse_click(ezgl::application* app, GdkEventButton* event, double x, double y);
void initial_setup(ezgl::application* app, bool new_window);

gboolean search_changed(GtkWidget* widget, gpointer data);
gboolean press_search(GtkWidget* widget, gpointer data);
gboolean open_another_map(GtkWidget* /*widget*/, gpointer data);

void POI_Toggle(GtkWidget* widget, ezgl::application* app);

void find_common_intersections_of_two_streets(GtkWidget* widget, ezgl::application* app);
void find_closest_POI_to_highlighted_intersection(GtkWidget* widget, ezgl::application* app);

// Function that converts a longitude to a cartesian x coordinate
double x_from_lon(float lon, double cos){
    return (kEarthRadiusInMeters * lon * kDegreeToRadian * cos);
}

// Function that converts a latitude to a cartesian y coordinate
double y_from_lat(float lat){
    return (kEarthRadiusInMeters * lat * kDegreeToRadian);
}

// Function that converts a cartesian x coordinate to a longitude
double lon_from_x(float x, double cos){
    return x / (kEarthRadiusInMeters * kDegreeToRadian * cos);
}

// Function that converts a cartesian y coordinate to a latitude
double lat_from_y(float y){
    return y / (kEarthRadiusInMeters * kDegreeToRadian);
}

void centre_map_at_a_point(double point_x, double point_y, ezgl::application* app){
    // Set the new displaying coordinates
    double new_window_x_min = 0;
    double new_window_y_min = 0;
    double new_window_x_max = 0;
    double new_window_y_max = 0;

    new_window_x_min = point_x - 600;
    new_window_y_min = point_y - 600;
    new_window_x_max = point_x + 600;
    new_window_y_max = point_y + 600;


    // Generate lower left and upper right coordinates for new displaying region
    ezgl::point2d new_window_lower_left(new_window_x_min, new_window_y_min);
    ezgl::point2d new_window_upper_right(new_window_x_max, new_window_y_max);
    ezgl::rectangle new_window(new_window_lower_left, new_window_upper_right);

    // Make map centre at the point and redraw
    std::string main_canvas_id = app -> get_main_canvas_id();
    auto canvas = app -> get_canvas(main_canvas_id);
    ezgl::zoom_fit(canvas, new_window);
}

void clear_globals(){
    xy_coordinate_version.clear();
    intersections.clear();

    osm_highway_motorway.clear();
    osm_highway_trunk.clear();
    osm_highway_primary.clear();
    osm_highway_secondary.clear();
    osm_highway_tertiary.clear();
    lowest_level_way.clear();
    special_way.clear();
    id_to_OSMWay.clear();
    lake_feature.clear();
    park_feature.clear();
    beach_feature.clear();
    river_feature.clear();
    island_feature.clear();
    building_feature.clear();
    greenspace_feature.clear();
    golfcourse_feature.clear();
    stream_feature.clear();
    glacier_feature.clear();
    building_area.clear();
    building_ordered.clear();
    xy_coor_subway.clear();
    
    //xy_lowest_level_way_point.clear();
    //xy_special_way_point.clear();

    
    segment_names.clear();
    connectedStreetSegments.clear();

    POIs.clear();
    intersectionNames.clear();
}

void find_map_bound_and_load_intersections(){
    // Initialize maximum and minimum longitude and latitude
    max_lat = getIntersectionPosition(0).latitude();
    min_lat = max_lat;
    max_lon = getIntersectionPosition(0).longitude();
    min_lon = max_lon;
    
    intersections.resize(getNumIntersections());
    
    // Loop through all intersections to find maximum and minimum longitude and latitude
    LatLon IntLatLonPos;
    for (IntersectionIdx id = 1; id < getNumIntersections(); id++) {
        IntLatLonPos = getIntersectionPosition(id);
        
        // Go through each intersection and find the max and min 
        max_lat = std::max (max_lat, IntLatLonPos.latitude());
        min_lat = std::min (min_lat, IntLatLonPos.latitude());
        max_lon = std::max (max_lon, IntLatLonPos.longitude());
        min_lon = std::min (min_lon, IntLatLonPos.longitude());
    }
    // Convert maximum and minimum longitude and latitude into cartesian coordinate
    Cos_LatLon_to_XY = cos (kDegreeToRadian * (max_lat + min_lat) / 2);
    x_min = x_from_lon(min_lon, Cos_LatLon_to_XY);
    x_max = x_from_lon(max_lon, Cos_LatLon_to_XY);
    y_min = y_from_lat(min_lat);
    y_max = y_from_lat(max_lat);
    
    // Set x and y coordinates, name, and highlight status for each intersection
    for (IntersectionIdx id = 0; id < getNumIntersections(); id++) {
        IntLatLonPos = getIntersectionPosition(id);
        intersections[id].xy_loc = make_point_xy(IntLatLonPos);
        intersections[id].name = getIntersectionName(id);
        intersections[id].highlight = false;
    }
}

void load_POIs(){
    POIs.resize(getNumPointsOfInterest());
    
    for (POIIdx id = 0; id < getNumPointsOfInterest(); id++){
        LatLon POILatLonPos = getPOIPosition(id);
        POIs[id].xy_loc = make_point_xy(POILatLonPos);
        POIs[id].name = getPOIName(id);
        POIs[id].type = getPOIType(id);
        POIs[id].highlight = false;
    }
}

bool rotation_adjust(double x0, double y0, double x1, double y1, double &rotation){
    if (x0 < x1 && y0 <= y1) return false;
    else if (x0 > x1 && y0 >= y1) return true;
    else if (x0 >= x1 && y0 < y1){
        rotation = -rotation;
        return true;
    }
    else if (x0 <= x1 && y0 > y1){
        rotation = -rotation;
        return true;
    }
    else return true;
}

double find_distance_between_two_point(double x0, double y0, double x1, double y1){
    double length;
    double deltaX = pow((x1 - x0), 2);
    double deltaY = pow((y1 - y0), 2);
    length = sqrt(deltaX + deltaY);
    return length;
}

void set_x_bond_and_center_and_rotation(int segID){
    double x_bound;
    double x0_coords, y0_coords, x1_coords, y1_coords;
    double x_center, y_center;
    double rotation;
    bool append_front = true;
    bool oneWay;
    std::string front = "<-- ";
    std::string back = " -->";

    x0_coords = segment_names[segID].from.x;
    y0_coords = segment_names[segID].from.y;

    x1_coords = segment_names[segID].to.x;
    y1_coords = segment_names[segID].to.y;

    x_center = x0_coords + (x1_coords - x0_coords)/2;
    y_center = y0_coords + (y1_coords - y0_coords)/2;

    x_bound = find_distance_between_two_point(x0_coords, y0_coords, x1_coords, y1_coords);

    if (x_bound != 0){
        rotation = acos(abs(x1_coords - x0_coords)/x_bound);
        rotation = rotation * 180 / M_PI;
        append_front = rotation_adjust(x0_coords, y0_coords, x1_coords, y1_coords, rotation);
    }
    else rotation = 0;
    ezgl::point2d center(x_center, y_center);

    oneWay = segment_names[segID].oneWay;
    if (append_front && oneWay) segment_names[segID].streetName = front.append(segment_names[segID].streetName);    
    else if (!append_front && oneWay) segment_names[segID].streetName = segment_names[segID].streetName.append(back);
    
    if (segment_names[segID].curved_pointes >= 10 && x_bound <= 20) x_bound *= 3;
    segment_names[segID].x_bound = x_bound;
    segment_names[segID].center = center;
    segment_names[segID].rotation = rotation;
}

void find_and_set_longest_straight_seg(int segID){
    int longest = 0;
    double lengthFinal, lengthTemp;
    ezgl::point2d temp_point0, temp_point1;

    //find first seg length
    temp_point0 = make_point_xy(getStreetSegmentCurvePoint(0, segID));
    lengthFinal = find_distance_between_two_point(segment_names[segID].from.x, segment_names[segID].from.y, temp_point0.x, temp_point0.y);

    //find last seg length
    temp_point0 = make_point_xy(getStreetSegmentCurvePoint(segment_names[segID].curved_pointes - 1, segID));
    lengthTemp = find_distance_between_two_point(segment_names[segID].to.x, segment_names[segID].to.y, temp_point0.x, temp_point0.y);

    //set longest length
    if (lengthTemp > lengthFinal) {
        lengthFinal = lengthTemp;
        longest = segment_names[segID].curved_pointes;
    }

    for (int curvePoints = 0; curvePoints < segment_names[segID].curved_pointes - 1; curvePoints++){
        temp_point0 = make_point_xy(getStreetSegmentCurvePoint(curvePoints, segID));
        temp_point1 = make_point_xy(getStreetSegmentCurvePoint(curvePoints + 1, segID));

        lengthTemp = find_distance_between_two_point(temp_point0.x, temp_point0.y, temp_point1.x, temp_point1.y);

        if (lengthTemp > lengthFinal) {
            lengthFinal = lengthTemp;
            longest = curvePoints + 1;
        }
    }

    if (longest == 0){
        temp_point0 = make_point_xy(getStreetSegmentCurvePoint(0, segID));
        segment_names[segID].to = temp_point0;
    }
    else if (longest == segment_names[segID].curved_pointes){
        temp_point0 = make_point_xy(getStreetSegmentCurvePoint(segment_names[segID].curved_pointes - 1, segID));
        segment_names[segID].from = temp_point0;
    }
    else {
        temp_point0 = make_point_xy(getStreetSegmentCurvePoint(longest - 1, segID));
        temp_point1 = make_point_xy(getStreetSegmentCurvePoint(longest, segID));
        segment_names[segID].from = temp_point0;
        segment_names[segID].to = temp_point1;
    }
}

void set_secondaryName_y_bound(){
    for (int segCounter = 0; segCounter < osm_highway_secondary.size(); segCounter++){
        segment_names[osm_highway_secondary[segCounter]].y_bound = 25;
    }
}

void set_tertiaryName_y_bound(){
    for (int segCounter = 0; segCounter < osm_highway_tertiary.size(); segCounter++){
        segment_names[osm_highway_tertiary[segCounter]].y_bound = 10;
    }
}

void set_smallName_y_bound(){
    for (int segCounter = 0; segCounter < lowest_level_way.size(); segCounter++){
        segment_names[lowest_level_way[segCounter]].y_bound = 10;
    }
    for (int segCounter = 0; segCounter < special_way.size(); segCounter++){
        segment_names[special_way[segCounter]].y_bound = 5;
    }
}

void load_all_segments_names(){
    bool oneWay, curved, draw;
    std::string unknow = "<unknown>";
    segment_names.resize(getNumStreetSegments());
    for (int segID = 0; segID < getNumStreetSegments(); segID++){
        StreetSegmentInfo segInfoTemp = getStreetSegmentInfo(segID);

        LatLon from_point = getIntersectionPosition (segInfoTemp.from);
        LatLon to_point = getIntersectionPosition (segInfoTemp.to);

        segment_names[segID].from = make_point_xy(from_point);
        segment_names[segID].to = make_point_xy(to_point);
        segment_names[segID].streetName = getStreetName(segInfoTemp.streetID);
        segment_names[segID].segID = segID;

        oneWay = segInfoTemp.oneWay;
        segment_names[segID].oneWay = oneWay;
        segment_names[segID].y_bound = 150;
        
        if (segInfoTemp.numCurvePoints != 0) curved = true;
        else curved = false;
        segment_names[segID].curved = curved;
        draw = true;
        if (segment_names[segID].streetName == unknow) draw = false;
        segment_names[segID].draw = draw;
        segment_names[segID].curved_pointes = segInfoTemp.numCurvePoints;
        if (!curved)set_x_bond_and_center_and_rotation(segID);
        else {
            find_and_set_longest_straight_seg(segID);
            set_x_bond_and_center_and_rotation(segID);
        }
    }
    set_smallName_y_bound();
    set_tertiaryName_y_bound();
    set_secondaryName_y_bound();
    //load_segments_to_nameOrderedMap();
    //disable_adjacent_segName();
    //load_segments_to_segIdOrderedSegementsMap();
}



ezgl::point2d make_point_xy (LatLon latlon) {
    float x_coor = x_from_lon(latlon.longitude(), Cos_LatLon_to_XY);
    float y_coor = y_from_lat(latlon.latitude());
    ezgl::point2d coor(x_coor, y_coor);
    return coor;
}

// Go through each street segments and pre-load coordinates into x-y form and store them into vector of structs
void coordinate_transformation () {
    for (StreetSegmentIdx street_segment_id = 0; street_segment_id < getNumStreetSegments(); street_segment_id++) {
        
        // Access information of street segment corresponding to its streetSegmentId
        StreetSegmentInfo streetSegmentInfo = getStreetSegmentInfo(street_segment_id);
        
        LatLon from_point = getIntersectionPosition (streetSegmentInfo.from);
        LatLon to_point = getIntersectionPosition (streetSegmentInfo.to);
        
        // Convert latitude and longitude into x and y coordinate using formula provided in m1
        xy_coordinate_version.resize (getNumStreetSegments());
        
        //xy_coordinate xy_coor;
        
        xy_coordinate_version[street_segment_id].from = make_point_xy (from_point);
        xy_coordinate_version[street_segment_id].to = make_point_xy (to_point);

    }
}

void load_specialway_point_xy (StreetSegmentIdx stree_segment_id) {
    
    std::vector <ezgl::point2d> segment_point_coor;
    xy_coordinate xy_coor = xy_coordinate_version[stree_segment_id];
    StreetSegmentInfo streetSegmentInfo = getStreetSegmentInfo (stree_segment_id);

    segment_point_coor.push_back(xy_coor.from);
    for (int curve_point = 0; curve_point < (streetSegmentInfo.numCurvePoints - 1); curve_point++) {
        LatLon ssCurvePoints = getStreetSegmentCurvePoint(curve_point, stree_segment_id);
        ezgl::point2d curvepoint_xy = make_point_xy (ssCurvePoints);
        segment_point_coor.push_back(curvepoint_xy);
    }    
    segment_point_coor.push_back(xy_coor.to);
    // after a full segment is formed and pushed into the vector

    xy_special_way_point.push_back (segment_point_coor);

    segment_point_coor.clear();
}

void load_lowestway_point_xy (StreetSegmentIdx stree_segment_id) {
    
    std::vector <ezgl::point2d> segment_point_coor;
    xy_coordinate xy_coor = xy_coordinate_version[stree_segment_id];
    StreetSegmentInfo streetSegmentInfo = getStreetSegmentInfo (stree_segment_id);

    segment_point_coor.push_back(xy_coor.from);
    for (int curve_point = 0; curve_point < (streetSegmentInfo.numCurvePoints - 1); curve_point++) {
        LatLon ssCurvePoints = getStreetSegmentCurvePoint(curve_point, stree_segment_id);
        ezgl::point2d curvepoint_xy = make_point_xy (ssCurvePoints);
        segment_point_coor.push_back(curvepoint_xy);
    }    
    segment_point_coor.push_back(xy_coor.to);
    // after a full segment is formed and pushed into the vector

    xy_lowest_level_way_point.push_back (segment_point_coor);

    segment_point_coor.clear();
}

void load_main_minor_road (){
    auto const start = std::chrono::high_resolution_clock::now();
    
    // load the hash map for the id_to_OSMWay
    for (int way_id = 0; way_id < getNumberOfWays(); way_id++) {
        
        const OSMWay* osmway = getWayByIndex (way_id);
        OSMID id = osmway -> id();
        
        id_to_OSMWay[id] = osmway;
    }
    
    // Loop through all the street segments to access its OSMWay* pointer by its wayID
    for (StreetSegmentIdx stree_segment_id = 0; stree_segment_id < getNumStreetSegments(); stree_segment_id++) {
        
        // Access its OSMWay* by wayID
        StreetSegmentInfo segmentInfo = getStreetSegmentInfo (stree_segment_id);
        const OSMWay* osmway = id_to_OSMWay[segmentInfo.wayOSMID];
        
        // look for its type and store it in vector if street segment matches any tag
        for (unsigned tag = 0; tag < getTagCount(osmway); tag++) {
            std::pair <std::string, std::string> tagPair = getTagPair(osmway, tag);
            if ((tagPair.first == "highway" && tagPair.second == "motorway") || (tagPair.first == "highway" && tagPair.second == "motorway_link")) {
                osm_highway_motorway.push_back(stree_segment_id); // load all the street segment id into the vector
            } else if ((tagPair.first == "highway" && tagPair.second == "trunk") || (tagPair.first == "highway" && tagPair.second == "trunk_link")) {
                osm_highway_trunk.push_back(stree_segment_id); // load all the street segment id into the vector
            } else if ((tagPair.first == "highway" && tagPair.second == "primary") || (tagPair.first == "highway" && tagPair.second == "primary_link")) {
                osm_highway_primary.push_back(stree_segment_id); // load all the street segment id into the vector
            } else if ((tagPair.first == "highway" &&tagPair.second == "secondary") || (tagPair.first == "highway" && tagPair.second == "secondary_link")) {
                osm_highway_secondary.push_back(stree_segment_id); // load all the street segment id into the vector
            } else if ((tagPair.first == "highway" &&tagPair.second == "tertiary") || (tagPair.first == "highway" && tagPair.second == "tertiary_link")) {
                osm_highway_tertiary.push_back(stree_segment_id); // load all the street segment id into the vector
            } else if (tagPair.first == "highway" && (tagPair.second == "living_street" || tagPair.second == "service" || tagPair.second == "pedestrian" 
                    || tagPair.second == "track" || tagPair.second == "bus_guideway" || tagPair.second == "escape" || tagPair.second == "raceway" || tagPair.second == "road" || tagPair.second == "busway")){
                load_specialway_point_xy(stree_segment_id);
                //special_way.push_back (stree_segment_id); // load all other street segment id into the vector
            } else {
                load_lowestway_point_xy(stree_segment_id);
            }
        }
    }
    auto const end = std::chrono::high_resolution_clock::now();
    auto const delta_time = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
    std::cout << "load_main_minor_road" << delta_time.count() << "s\n";
}

/*void pre_load_world_data () {
    xy_all_feature_point.resize (getNumFeatures());
    xy_all_segment_point.resize (getNumStreetSegments());
    
    for (FeatureIdx feature_id = 0; feature_id < getNumFeatures(); feature_id++) {
        
        std::vector <ezgl::point2d> feature_point_coor;
        int num_feature_points = getNumFeaturePoints (feature_id);
        feature_point_coor.resize(num_feature_points);

        for (int feature_point = 0; feature_point < num_feature_points; feature_point++) {

            LatLon feature_latlon = getFeaturePoint (feature_point, feature_id);

            ezgl::point2d feature_point_xy = make_point_xy (feature_latlon);
            feature_point_coor.push_back (feature_point_xy);
        }
        xy_all_feature_point[feature_id] = feature_point_coor;
        feature_point_coor.clear();
    }
    
    
    for (StreetSegmentIdx segment_id = 0; segment_id < getNumStreetSegments(); segment_id++) {
        std::vector <ezgl::point2d> segment_point_coor;
        
        xy_coordinate xy_coor = xy_coordinate_version[segment_id];
        StreetSegmentInfo streetSegmentInfo = getStreetSegmentInfo (segment_id);
        
        segment_point_coor.push_back(xy_coor.from);
        for (int curve_point = 0; curve_point < (streetSegmentInfo.numCurvePoints - 1); curve_point++) {
            LatLon ssCurvePoints = getStreetSegmentCurvePoint(curve_point, segment_id);
            ezgl::point2d curvepoint_xy = make_point_xy (ssCurvePoints);
            segment_point_coor.push_back(curvepoint_xy);
        }
        segment_point_coor.push_back(xy_coor.to);
        xy_all_segment_point[segment_id] = segment_point_coor;
        segment_point_coor.clear();
    }
}*/

void draw_one_street_segment_smallway (ezgl::renderer *g, std::vector<std::vector<ezgl::point2d>> xy_point, int red, int green, int blue, int opaque, int line_width) {
    
    for (int street_seg_num = 0; street_seg_num <  xy_point.size(); street_seg_num++) {
        
        std::vector<ezgl::point2d> xy_coor = xy_point[street_seg_num];
        
        int num_point_in_range = 0;
        for (int point_num = 0; point_num < xy_coor.size(); point_num++) {
            if (g -> get_visible_world().contains(xy_coor[point_num])) {
                num_point_in_range++;
            }
        }
        // If more than half of the points of segment in the range, draw it
        if (num_point_in_range > 0) {//xy_point[street_seg_num].size() / 2) {
            num_point_in_range = 0; // reset counter for next segment
            
            for (int point_num = 0; point_num < xy_coor.size() - 1; point_num++) {
                g -> set_color (red, green, blue, opaque);
                g -> set_line_width (line_width);
                g -> draw_line(xy_coor[point_num], xy_coor[point_num + 1]);
            }
        }
        xy_coor.clear();
    }
}

void draw_one_street_segment (ezgl::renderer *g, StreetSegmentIdx segment_id, int red, int green, int blue, int opaque, int line_width) {
    
    //for (int feature_id = 0; feature_id < )
    
    
    xy_coordinate xy_coor = xy_coordinate_version[segment_id];
    StreetSegmentInfo streetSegmentInfo = getStreetSegmentInfo (segment_id); 
    if (streetSegmentInfo.numCurvePoints == 0) {
            
        // Have no curve point, directly draw from "from" to "to" point
        g -> set_color (red, green ,blue, opaque);
        g -> set_line_width(line_width);
        g -> set_line_cap(ezgl::line_cap::round);
        g -> draw_line(xy_coor.from, xy_coor.to);
    } else if (streetSegmentInfo.numCurvePoints == 1) {

        // Only have 1 curve point, draw two lines 
        LatLon ssCurvePoints = getStreetSegmentCurvePoint((streetSegmentInfo.numCurvePoints - 1), segment_id);
        g -> set_color (red, green ,blue, opaque);
        g -> set_line_width(line_width);
        g -> set_line_cap(ezgl::line_cap::round);
        g -> draw_line(xy_coor.from, make_point_xy (ssCurvePoints));
        g -> draw_line(make_point_xy (ssCurvePoints), xy_coor.to);
    } else {

        // More than 1 curve point 
        for (int curve_point = 0; curve_point < (streetSegmentInfo.numCurvePoints - 1); curve_point++) {
            
            // Get LatLon for first curve point and next curve point 
            LatLon ssCurvePoints = getStreetSegmentCurvePoint(curve_point, segment_id);
            LatLon ssCurvePoints_next = getStreetSegmentCurvePoint((curve_point + 1), segment_id);

            g -> set_color (red, green ,blue, opaque);
            g -> set_line_width(line_width);
            g -> set_line_cap(ezgl::line_cap::round);
            g -> draw_line(make_point_xy (ssCurvePoints), make_point_xy (ssCurvePoints_next));
        } 

        // Get the latLon for the first and last curve point
        LatLon firstPoint = getStreetSegmentCurvePoint(0, segment_id); 
        LatLon lastPoint = getStreetSegmentCurvePoint((streetSegmentInfo.numCurvePoints - 1), segment_id);
        
        // Draw the line other than the line between curve points
        g -> set_color (red, green ,blue, opaque);
        g -> set_line_width(line_width);
        g -> set_line_cap(ezgl::line_cap::round);
        g -> draw_line(xy_coor.from, make_point_xy (firstPoint));
        g -> draw_line(make_point_xy (lastPoint), xy_coor.to);
    }
}

void draw_one_segment_name(ezgl::renderer *g, StreetSegmentIdx segID){
    if (segment_names[segID].draw){
        g ->set_color(0, 0, 0);
        g ->set_text_rotation(segment_names[segID].rotation);
        g ->set_font_size(12);
        g ->draw_text(segment_names[segID].center, segment_names[segID].streetName, segment_names[segID].x_bound, segment_names[segID].y_bound);
    }
}


void draw_main_minor_road (ezgl::renderer *g) {
    auto const start = std::chrono::high_resolution_clock::now();
    
    // Draw all the lowest level way 
    if (level_of_detail < 1) {
            draw_one_street_segment_smallway (g, xy_special_way_point, 214, 214, 206, 255, 0);
            //draw_one_segment_name (g, special_way[segment_id]);
    }
    
    if (level_of_detail < 2) {

            draw_one_street_segment_smallway (g, xy_lowest_level_way_point, 214, 214, 206, 255, 0);
            //draw_one_segment_name (g, lowest_level_way[segment_id]);
    }
    
    // Draw tertiary_way
    if (level_of_detail < 6) {
        for (int segment_id = 0; segment_id < osm_highway_tertiary.size(); segment_id++) {

            draw_one_street_segment (g, osm_highway_tertiary[segment_id], 240, 233, 31, 255, 2);
            draw_one_segment_name (g, osm_highway_tertiary[segment_id]);
        }
    }
    
    // Draw secondary_way
    if ((osm_highway_primary.size() + osm_highway_trunk.size() + osm_highway_motorway.size()) < 100000 || (level_of_detail < 7)) {
        if (level_of_detail < 8 ) {
            auto const start_2 = std::chrono::high_resolution_clock::now();
            
            for (int segment_id = 0; segment_id < osm_highway_secondary.size(); segment_id++) {
                
                draw_one_street_segment (g, osm_highway_secondary[segment_id], 247, 188, 25, 255, 2);
                draw_one_segment_name (g, osm_highway_secondary[segment_id]);
                
            }
            
            auto const end = std::chrono::high_resolution_clock::now();
            auto const delta_time = std::chrono::duration_cast<std::chrono::duration<double>> (end - start_2);
            std::cout << "draw_secondary_road_and_names" << delta_time.count() << "s\n";
        }
    }
    
    if ((osm_highway_primary.size() + osm_highway_trunk.size() + osm_highway_motorway.size()) < 30000 || (level_of_detail < 8)) {
        if (level_of_detail < 8 || level_of_detail == 8) {
            for (int segment_id = 0; segment_id < osm_highway_primary.size(); segment_id++) {

                draw_one_street_segment (g, osm_highway_primary[segment_id], 227, 104, 143, 255, 5);
                draw_one_segment_name (g, osm_highway_primary[segment_id]);
            }
        }
    }
    
    if ((osm_highway_primary.size() + osm_highway_trunk.size() + osm_highway_motorway.size()) < 40000 || (level_of_detail < 8)) {
        if (level_of_detail < 8 || level_of_detail == 8) {
            for (int segment_id = 0; segment_id < osm_highway_trunk.size(); segment_id++) {

                draw_one_street_segment (g, osm_highway_trunk[segment_id], 227, 104, 143, 255, 5);
                draw_one_segment_name (g, osm_highway_trunk[segment_id]);
            }
        }
    }
    
    // Draw motorway 
    if (level_of_detail < 8 || level_of_detail == 8) {
        for (int segment_id = 0; segment_id < osm_highway_motorway.size(); segment_id++) {
            draw_one_street_segment (g, osm_highway_motorway[segment_id], 227, 104, 143, 255, 5);
            draw_one_segment_name (g, osm_highway_motorway[segment_id]);
        }
    }
    
    auto const end = std::chrono::high_resolution_clock::now();
    auto const delta_time = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
    std::cout << "draw_main_minor_road" << delta_time.count() << "s\n";
}



void sort_building_by_area (std::vector<FeatureIdx> building) {
    // load the area into multi-map
    for (FeatureIdx feature_id = 0; feature_id < building.size(); feature_id++) {
        double area = findFeatureArea(building[feature_id]);
        building_area.insert(std::make_pair(area, building[feature_id]));
    }

    for (auto it = building_area.rbegin(); it != building_area.rend(); it++) {
        building_ordered.push_back (it -> second);
    }
}

void load_subway () {
    auto const start = std::chrono::high_resolution_clock::now();
    
    std::vector<const OSMRelation*> subway_lines;
    for (unsigned num_relations = 0; num_relations < getNumberOfRelations(); num_relations++) {
        
        const OSMRelation* current_Rel = getRelationByIndex (num_relations);
        
        for (unsigned num_tag = 0; num_tag < getTagCount (current_Rel); num_tag++ ){
            std::pair <std::string, std::string> tagPair = getTagPair(current_Rel, num_tag);
            
            if (tagPair.first == "route" && tagPair.second == "subway") {
                subway_lines.push_back(current_Rel);
                break;
            }
        }
    }
    
    xy_coor_subway.resize(subway_lines.size());
    for (unsigned subway_lines_index = 0; subway_lines_index < subway_lines.size(); subway_lines_index++) {
        std::vector <TypedOSMID> route_members = getRelationMembers (subway_lines[subway_lines_index]);
        
        std::vector<ezgl::point2d> subway_node_xy;
        for (int route_members_index = 0; route_members_index < route_members.size(); route_members_index++) {
            if (route_members[route_members_index].type() == TypedOSMID::Node) {
                
                const OSMNode* current_node = nullptr;

                for (unsigned num_node = 0; num_node < getNumberOfNodes(); num_node++) {
                    current_node = getNodeByIndex (num_node);
                    if (current_node -> id() == route_members[route_members_index]) {
                        
                        LatLon node_latlon = getNodeCoords(current_node);
                        ezgl::point2d xy_coor_node = make_point_xy (node_latlon);
                        subway_node_xy.push_back(xy_coor_node);
                        break;
                    }
                }
            }
        } 
        xy_coor_subway[subway_lines_index] = subway_node_xy;
        subway_node_xy.clear();
    }
    subway_lines.clear();
    
    auto const end = std::chrono::high_resolution_clock::now();
    auto const delta_time = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
    std::cout << "load_subway" << delta_time.count() << "s\n";
}

void load_feature_points() {
    auto const start = std::chrono::high_resolution_clock::now();
    for (FeatureIdx feature_id = 0; feature_id < getNumFeatures(); feature_id++) {
        FeatureType type = getFeatureType (feature_id);
        
        if (type == LAKE) {
            lake_feature.push_back (feature_id);
        }
        
        if (type == PARK) {
            park_feature.push_back (feature_id);
        }
        
        if (type == BEACH) {
            beach_feature.push_back (feature_id);
        }

        if (type == ISLAND) {
            island_feature.push_back (feature_id);
        }
        
        if (type == BUILDING) {
            building_feature.push_back (feature_id);
        }
        
        if (type == GREENSPACE) {
            greenspace_feature.push_back (feature_id);
        }
        
        if (type == GOLFCOURSE) {
            golfcourse_feature.push_back (feature_id);
        }
        
        if (type == STREAM) {
            stream_feature.push_back (feature_id);
        }
        
        if (type == RIVER) {
            river_feature.push_back (feature_id);
        }
        
        if (type == GLACIER) {
            glacier_feature.push_back (feature_id);
        }
    }
    sort_building_by_area (building_feature);
    
    auto const end = std::chrono::high_resolution_clock::now();
    auto const delta_time = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
    std::cout << "load_feature_points" << delta_time.count() << "s\n";
}

void draw_different_feature_area(ezgl::renderer *g, std::vector<FeatureIdx> feature_vector, int red, int green, int blue, int line_width) {
    std::vector <ezgl::point2d> feature_point_coor;
    // Get the feature type and check it.
    for (int feature_num = 0; feature_num < feature_vector.size(); feature_num++) {
        FeatureIdx feature_id = feature_vector[feature_num];
        // Get the number of feature points and create a vector to store x,y coordinates of feature points
        
        int num_feature_points = getNumFeaturePoints (feature_id);
        if (num_feature_points == 1) {
            /*LatLon feature_latlon = getFeaturePoint (0, feature_id);
            ezgl::point2d feature_point_xy = make_point_xy (feature_latlon);
            g -> set_color (ezgl::PINK);
            g -> draw_arc (feature_point_xy, 5, 0, 255);*/
            continue;
        }

        feature_point_coor.resize(num_feature_points);

        for (int feature_point = 0; feature_point < num_feature_points; feature_point++) {

            LatLon feature_latlon = getFeaturePoint (feature_point, feature_id);

            ezgl::point2d feature_point_xy = make_point_xy (feature_latlon);
            feature_point_coor[feature_point] = feature_point_xy;
        }

        LatLon beginPoint = getFeaturePoint(0, feature_id);
        LatLon endPoint = getFeaturePoint (num_feature_points - 1, feature_id);

        if (beginPoint.latitude() != endPoint.latitude()){
            g -> set_line_width (line_width);
            g -> set_color (red, green, blue);
            for (int feature_point_num = 0; feature_point_num < feature_point_coor.size() - 1; feature_point_num++) {
                //g -> set_line_cap(ezgl::line_cap::round);
                g -> draw_line (feature_point_coor[feature_point_num], feature_point_coor[feature_point_num + 1]);
            }
            //feature_point_coor_lake.clear();
        } else {
            g -> set_color (red, green, blue);
            g -> fill_poly (feature_point_coor);
            //feature_point_coor_lake.clear();
        }
    }
}

void draw_building_first_layer_area(ezgl::renderer *g, std::vector<FeatureIdx> building_order, int red, int green, int blue, int line_width, int size){
    std::vector <ezgl::point2d> feature_point_coor;
    // Get the feature type and check it.
    for (int feature_num = 0; feature_num < size; feature_num++) {
        FeatureIdx feature_id = building_order[feature_num];
        // Get the number of feature points and create a vector to store x,y coordinates of feature points
        
        int num_feature_points = getNumFeaturePoints (feature_id);
        if (num_feature_points == 1) {
            /*LatLon feature_latlon = getFeaturePoint (0, feature_id);
            ezgl::point2d feature_point_xy = make_point_xy (feature_latlon);
            g -> set_color (ezgl::PINK);
            g -> draw_arc (feature_point_xy, 0, 0, 255);*/
            continue;
        }

        feature_point_coor.resize(num_feature_points);

        for (int feature_point = 0; feature_point < num_feature_points; feature_point++) {

            LatLon feature_latlon = getFeaturePoint (feature_point, feature_id);

            ezgl::point2d feature_point_xy = make_point_xy (feature_latlon);
            feature_point_coor[feature_point] = feature_point_xy;
        }

        LatLon beginPoint = getFeaturePoint(0, feature_id);
        LatLon endPoint = getFeaturePoint (num_feature_points - 1, feature_id);

        if (beginPoint.latitude() != endPoint.latitude()){
            g -> set_line_width (line_width);
            g -> set_color (red, green, blue);
            for (int feature_point_num = 0; feature_point_num < feature_point_coor.size() - 1; feature_point_num++) {
                //g -> set_line_cap(ezgl::line_cap::round);
                g -> draw_line (feature_point_coor[feature_point_num], feature_point_coor[feature_point_num + 1]);
            }
            //feature_point_coor_lake.clear();
        } else {
            g -> set_color (red, green, blue);
            g -> fill_poly (feature_point_coor);
            //feature_point_coor_lake.clear();
        }
    }
}

void draw_feature_area (ezgl::renderer *g) {
    auto const start = std::chrono::high_resolution_clock::now();
    
    if (level_of_detail < 8 || level_of_detail == 8) {
        ezgl::rectangle current_world = g-> get_visible_world();
        
        std::vector<ezgl::point2d> xy_coor;
        ezgl::point2d left_top (current_world.left(), current_world.top());
        xy_coor.push_back (left_top);
        ezgl::point2d left_bottom (current_world.left(), current_world.bottom());
        xy_coor.push_back (left_bottom);
        ezgl::point2d right_bottom (current_world.right(), current_world.bottom());
        xy_coor.push_back (right_bottom);
        ezgl::point2d right_top (current_world.right(), current_world.top());
        xy_coor.push_back (right_top);
        xy_coor.push_back (left_top); // push the first on again to form a closed polygon
        
        g -> set_color (247, 247, 228);
        g -> fill_poly (xy_coor);
        xy_coor.clear();
    }
    
    if (level_of_detail < 8 || level_of_detail == 8) {
        draw_different_feature_area(g, lake_feature, 167, 205, 242, 4);
    }
    
    if (level_of_detail < 8 || level_of_detail == 8) {
        draw_different_feature_area(g, island_feature, 255, 204, 0, 4);
    }
    
    if (level_of_detail < 8 || level_of_detail == 8) {
        draw_different_feature_area(g, beach_feature, 255, 224, 71, 4);
    }
    
    if (level_of_detail < 8 || level_of_detail == 8) {
        draw_different_feature_area(g, park_feature, 187, 218, 164, 4);
    }
    
    if (level_of_detail < 8 || level_of_detail == 8) {
        draw_different_feature_area(g, glacier_feature, 0, 255, 213, 4);
    }
    
    if (level_of_detail < 6) {
        draw_different_feature_area(g, greenspace_feature, 30, 163, 98, 4);
    }
    
    if (level_of_detail < 4) {
        draw_different_feature_area(g, golfcourse_feature, 125, 184, 0, 4);
    }
    
    if (level_of_detail <= 8) {
        draw_different_feature_area(g, stream_feature, 155, 191, 244, 1);
    }
    
    if (level_of_detail <= 8) {
        draw_different_feature_area(g, river_feature, 155, 191, 244, 5);
    }
    
    if (level_of_detail == 3) {
        draw_building_first_layer_area (g, building_ordered, 128, 128, 128, 2, (building_ordered.size() / 8));
    }
    
    if (level_of_detail < 3) {
        draw_building_first_layer_area (g, building_ordered, 128, 128, 128, 2, (building_ordered.size() / 4));
    }
    
    if (level_of_detail < 1) {
        draw_different_feature_area (g, building_feature, 128, 128, 128, 2);
    }
    
    auto const end = std::chrono::high_resolution_clock::now();
    auto const delta_time = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
    std::cout << "draw_feature_area" << delta_time.count() << "s\n";
}

void update_level_of_detail (ezgl::renderer *g) {
    ezgl::rectangle current_world = g -> get_visible_world();
    
    double area = abs (current_world.area());
    if ((area / max_area) > 1 || (area / max_area) == 1) {
        level_of_detail = 8;
    }
    
    if ((area / max_area) < 0.5) {
        level_of_detail = 7;
    }
    
    if ((area / max_area) < 0.1 ) {
        level_of_detail = 6;
    }
    
    if ((area / max_area) < 0.05 ) {
        level_of_detail = 5;
    }
    
    if ((area / max_area) < 0.01) {
        level_of_detail = 4;
    }
    
    if ((area / max_area) < 0.005 ) {
        level_of_detail = 3;
    }
    
    if ((area / max_area) < 0.0005 ) {
        level_of_detail = 2;
    }
    
    if ((area / max_area) < 0.0001 ) {
        level_of_detail = 1;
    }
    
    if ((area / max_area) < 0.00001 ) {
        level_of_detail = 0;
    }
}

void draw_highlighted_intersections(ezgl::renderer *g){
    // Display highlighted intersection in red
    g -> set_color(ezgl::RED);
    
    // Find the highlighted intersection and display
    for(int id = 0; id < intersections.size(); id++){
        if(intersections[id].highlight)
            g -> fill_arc(intersections[id].xy_loc, 15, 0, 360);
    }
}

void draw_highlighted_POIs(ezgl::renderer *g){
    // Display highlighted intersection in red
    g -> set_color(ezgl::PURPLE);
    
    // Find the highlighted intersection and display
    for(int id = 0; id < POIs.size(); id++){
        if(POIs[id].highlight)
            g -> fill_arc(POIs[id].xy_loc, 15, 0, 360);
    }
}

void draw_all_POIs(ezgl::renderer *g){
    if(Display_All_POIs){
        g -> set_color(ezgl::PLUM);
        for(int poi = 0; poi < POIs.size(); poi++)
            g -> fill_arc(POIs[poi].xy_loc, 10, 0, 360);
    }
}

void draw_subway (ezgl::renderer *g) {
    auto const start = std::chrono::high_resolution_clock::now();
    
    if(Display_All_Subways) {
        for (int subway_line_number = 0; subway_line_number < xy_coor_subway.size(); subway_line_number++) {
            g -> set_color (ezgl::BLUE);
            g -> set_line_width (2);
            for (int subway_single_line = 0; subway_single_line < xy_coor_subway[subway_line_number].size() - 1; subway_single_line ++) {
                g -> draw_line (xy_coor_subway[subway_line_number][subway_single_line], xy_coor_subway[subway_line_number][subway_single_line + 1]);
            }
        }
    }
    auto const end = std::chrono::high_resolution_clock::now();
    auto const delta_time = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
    std::cout << "draw_subway" << delta_time.count() << "s\n";
}

void draw_main_canvas (ezgl::renderer *g) {
    auto const start = std::chrono::high_resolution_clock::now();
    
    update_level_of_detail (g);
    
    draw_feature_area (g);
    //draw_all_street_segments (g);
    g ->format_font(FONTS[language], ezgl::font_slant::normal, ezgl::font_weight::normal);
    draw_main_minor_road (g);
    
    draw_subway (g);
    
    draw_all_POIs(g);
    
    draw_highlighted_intersections(g);
    
    draw_highlighted_POIs(g);
    
    auto const end = std::chrono::high_resolution_clock::now();
    auto const delta_time = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
    std::cout << "draw_main_canvas" << delta_time.count() << "s\n";
}

void drawMap() {
    
    auto const start = std::chrono::high_resolution_clock::now();
    ezgl::application::settings settings;
    settings.main_ui_resource = "libstreetmap/resources/main.ui";
    settings.window_identifier = "MainWindow";
    settings.canvas_identifier = "MainCanvas";
    
    ezgl::application application (settings);
    
    find_map_bound_and_load_intersections();
    
    Display_All_POIs = false;
    
    loadIntersectionNames();
    
    load_POIs();
    
    load_subway();
    
    coordinate_transformation();
    
    load_main_minor_road();

    load_all_segments_names();
    
    load_feature_points();
    
    ezgl::rectangle initial_world ({x_min, y_min}, {x_max, y_max});
    max_area = abs (initial_world.area());
    
    auto const end = std::chrono::high_resolution_clock::now();
    auto const delta_time = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
    std::cout << "load_total" << delta_time.count() << "s\n";
    
    application.add_canvas ("MainCanvas", draw_main_canvas, initial_world);
    
    application.run (initial_setup, act_on_mouse_click, nullptr, nullptr);
    
    clear_globals();
}

void act_on_mouse_click(ezgl::application* app, GdkEventButton* /*event*/, double x, double y){
    LatLon pos = LatLon(lat_from_y(y), lon_from_x(x, Cos_LatLon_to_XY));
    int id = findClosestIntersection(pos);
    
    // Reset highlight status of POIs to false
    for(int pt_of_interest = 0; pt_of_interest < POIs.size(); pt_of_interest++)
        POIs[pt_of_interest].highlight = false;
    
    // Reset all other intersection highlights to false but invert the highlight status of the selected one
    for(int intersection = 0; intersection < intersections.size(); intersection++){
        if(intersection == id)
            intersections[intersection].highlight = !intersections[intersection].highlight;
        else
            intersections[intersection].highlight = false;
    }
    
    if(intersections[id].highlight == true){
        std::stringstream message;
        message << "Intersection Selected: " << intersections[id].name;
        app -> update_message(message.str());
    }
    
    else{
        std::stringstream message;
        message << "";
        app -> update_message(message.str());
    }
    
    app -> refresh_drawing();
}

void initial_setup(ezgl::application* app, bool /*new_window*/){
    search_entry = app->get_object("SearchEntry");
    g_signal_connect(search_entry, "changed", G_CALLBACK(search_changed), app);
    
    GObject *search_button = app->get_object("SearchButton");
    g_signal_connect(search_button, "clicked", G_CALLBACK(press_search), app);
    
    change_map = app->get_object("MapSelect");
    g_signal_connect(change_map, "changed", G_CALLBACK(open_another_map), app);
    
    app -> create_button("POI OFF", 7, POI_Toggle);
    app -> create_button("Subway Lines OFF", 8, draw_subway_lines);
    app -> create_button("Find Common Ints", 9, find_common_intersections_of_two_streets);
    app -> create_button("Find Closest POI", 10, find_closest_POI_to_highlighted_intersection);
}

void draw_subway_lines(GtkWidget* /*widget*/, ezgl::application* app) {
    Display_All_Subways = !Display_All_Subways;
    std::stringstream message;
    if(Display_All_Subways){
        app->change_button_text("Subway Lines OFF", "Subway Lines ON");
        
        message << "Subway routes of " << Map_Name << " is shown";
        app -> update_message(message.str());
    }
    else{
        app->change_button_text("Subway Lines ON", "Subway Lines OFF");
        message << "Subway routes of " << Map_Name << " is removed";
        app -> update_message(message.str());
    }
    app -> refresh_drawing();
}

gboolean search_changed(GtkWidget* /*widget*/, gpointer data){
    
    // Get the application object, text entry, and list store objects
    auto app = static_cast<ezgl::application *>(data);
    text_entry = (GtkEntry*) app -> get_object("SearchEntry");
    GtkListStore *store = (GtkListStore*) app -> get_object("ListStore");
    
    // Get entry of search bar as string
    std::string user_input = gtk_entry_get_text(text_entry);
    
    gtk_list_store_clear(store);
    GtkTreeIter iter;
    
    auto intersectionIDs = findIntersectionIdsFromPartialIntersectionName(user_input);
    
    for(int id = 0; id < intersectionIDs.size(); id++){
        std::string intersectionName = getIntersectionName(intersectionIDs[id]);
        
        gchar* intersection_name_char_version = new char[intersectionName.length() + 1];
        strncpy(intersection_name_char_version, intersectionName.c_str(), intersectionName.length() + 1);
        
        gtk_list_store_append(store, &iter);
        
        gtk_list_store_set(store, &iter, 0, intersection_name_char_version, -1);
        
        delete [] intersection_name_char_version;
    }

    return true;
}

gboolean press_search(GtkWidget* /*widget*/, gpointer data){
    // Get the text from search entry and the application object
    auto app = static_cast<ezgl::application *>(data);
    std::string intersectionName = gtk_entry_get_text(GTK_ENTRY(search_entry));
    
    // Check if it is a valid intersection name by looping through all intersection names
    int matched_intersection_id = -1;
    for(int id = 0; id < intersections.size(); id++){
        if(intersectionName == intersections[id].name){
            matched_intersection_id = id;
            break;
        }
    }
    
    std::stringstream message;
    
    // If it is invalid, tell the user it's invalid and do nothing
    if(matched_intersection_id == -1){
        message << intersectionName << " is not a valid intersection name";
        app -> update_message(message.str());
    }
    
    // Clear the highlight status of all intersections and POIs and highlight the selected intersection
    else{
        // Reset highlight status of POIs to false
        for(int pt_of_interest = 0; pt_of_interest < POIs.size(); pt_of_interest++)
            POIs[pt_of_interest].highlight = false;

        // Reset all other intersection highlights to false but true for the selected one
        for(int intersection = 0; intersection < intersections.size(); intersection++){
            if(intersection == matched_intersection_id)
                intersections[intersection].highlight = true;
            else
                intersections[intersection].highlight = false;
        }
        
        // Update Intersection info and center the map at it
        message << "Intersection Selected: " << intersections[matched_intersection_id].name;
        app -> update_message(message.str());
        centre_map_at_a_point(intersections[matched_intersection_id].xy_loc.x, intersections[matched_intersection_id].xy_loc.y, app);
    }
    
    return true;
}

gboolean open_another_map(GtkWidget* /*widget*/, gpointer data){
    // Get the text from search entry and the application object
    auto app = static_cast<ezgl::application *>(data);
    Map_Name = gtk_combo_box_text_get_active_text((GtkComboBoxText*)change_map);
    
    // Parse path to map
    std::string map_path = "/cad2/ece297s/public/maps/" + Map_Name + ".streets.bin";
    
    // Clear global variables, close the current map, and close the window
    clear_globals();
    closeMap();
    
    // The following line is added to EZGL library application::quit() function to forcibly close the window:
    // gtk_widget_destroy((GtkWidget*)get_object(m_window_id.c_str()));
    app -> quit();
    
    bool load_success = loadMap(map_path);
    if(!load_success)
        std::cerr << "Failed to load map '" << map_path << std::endl;

    std::cout << "Successfully loaded map '" << map_path << std::endl;
    
    if (Map_Name == "tokyo_japan") language = 1;
    else if (Map_Name == "beijing_china") language = 2;
    else language = 0;
    
    drawMap();
    
    return true;
}

void POI_Toggle(GtkWidget* /*widget*/, ezgl::application* app){
    Display_All_POIs = !Display_All_POIs;
    std::stringstream message;
    if(Display_All_POIs){
        app->change_button_text("POI OFF", "POI ON");
        message << "POIs of " << Map_Name << " is shown";
        app -> update_message(message.str());
    }
    else{
        app->change_button_text("POI ON", "POI OFF");
        message << "POIs of " << Map_Name << " is removed";
        app -> update_message(message.str());
    }
    app -> refresh_drawing();
}

// Functions that finds and displays common intersections of two streets inputed by user, controlled by "Find Common Ints" Button
void find_common_intersections_of_two_streets(GtkWidget* /*widget*/, ezgl::application* app){
    
    // Reset highlight of all intersections and POIs to false
    for(int intersection = 0; intersection < intersections.size(); intersection++)
        intersections[intersection].highlight = false;
    
    for(int pt_of_interest = 0; pt_of_interest < POIs.size(); pt_of_interest++)
        POIs[pt_of_interest].highlight = false;
    
    // Initializing Variables
    std::string street1_input_name;
    int street1_name_selection;
    std::string street1_name;
    
    std::string street2_input_name;
    int street2_name_selection;
    std::string street2_name;
    
    // Ask user to input street name #1
    std::cout << "Please Enter Street Name #1 and then press <Enter>: ";
    std::cin >> street1_input_name;
    auto possible_street1_ids = findStreetIdsFromPartialStreetName(street1_input_name);
    
    // Check for invalid input
    while(possible_street1_ids.size() == 0){
        std::cout << "Invalid Input, Please Enter Street Name #1 Again: ";
        std::cin >> street1_input_name;
        possible_street1_ids = findStreetIdsFromPartialStreetName(street1_input_name);
    }
    
    // Display all possible street names
    std::cout << "Street Names You May Be Looking For:" << std::endl;
    int num_of_entry_per_line = 4;
    for(int street = 0; street < possible_street1_ids.size(); street++){
        std::cout << street + 1 << ". " << getStreetName(possible_street1_ids[street]) << "    ";
        num_of_entry_per_line--;
        if(num_of_entry_per_line == 0){
            num_of_entry_per_line = 4;
            std::cout << std::endl;
        }
    }
    
    if(num_of_entry_per_line != 4)
        std::cout << std::endl;
    
    // Ask user to select the wanted street name
    std::cout << "Please Enter the Number Corresponding to the Desired Street Name #1 and then press <Enter>: ";
    std::cin >> street1_name_selection;
    
    // Check for invalid input
    while(street1_name_selection < 1 || street1_name_selection > possible_street1_ids.size() || std::cin.fail()) {
        std::cout << "Invalid Input, Please Enter the Number Corresponding to the Desired Street Name #1 Again: ";
        std::cin.clear();
        std::cin.ignore(256,'\n');
        std::cin >> street1_name_selection;
    }
    street1_name = getStreetName(possible_street1_ids[street1_name_selection - 1]);
    
    // Ask user to input street name #2
    std::cout << "Please Enter Street Name #2 and then press <Enter>: ";
    std::cin >> street2_input_name;
    auto possible_street2_ids = findStreetIdsFromPartialStreetName(street2_input_name);
    
    // Check for invalid input
    while(possible_street2_ids.size() == 0){
        std::cout << "Invalid Input, Please Enter Street Name #2 Again: ";
        std::cin >> street2_input_name;
        possible_street2_ids = findStreetIdsFromPartialStreetName(street2_input_name);
    }
    
    // Display all possible street names
    std::cout << "Street Names You May Be Looking For:" << std::endl;
    num_of_entry_per_line = 4;
    for(int street = 0; street < possible_street2_ids.size(); street++){
        std::cout << street + 1 << ". " << getStreetName(possible_street2_ids[street]) << "    ";
        num_of_entry_per_line--;
        if(num_of_entry_per_line == 0){
            num_of_entry_per_line = 4;
            std::cout << std::endl;
        }
    }
    if(num_of_entry_per_line != 4)
        std::cout << std::endl;
    
    // Ask user to select the wanted street name
    std::cout << "Please Enter the Number Corresponding to the Desired Street Name #2 and then press <Enter>: ";
    std::cin >> street2_name_selection;
    
    // Check for invalid input
    while(street2_name_selection < 1 || street2_name_selection > possible_street2_ids.size() || std::cin.fail()) {
        std::cout << "Invalid Input, Please Enter the Number Corresponding to the Desired Street Name #2 Again: ";
        std::cin.clear();
        std::cin.ignore(256,'\n');
        std::cin >> street2_name_selection;
    }
    street2_name = getStreetName(possible_street2_ids[street2_name_selection - 1]);
    
    // Show user the two selected street names and their common intersections
    std::cout << "The Two Selected Street Names are: " << street1_name << " and " << street2_name << "." << std::endl;
    auto common_intersections = findIntersectionsOfTwoStreets(std::make_pair(possible_street1_ids[street1_name_selection - 1], 
                                                                             possible_street2_ids[street2_name_selection - 1]));
    std::stringstream message;
    
    // If vector size = 0, then no common intersections
    if(common_intersections.size() == 0){
        std::cout << "Their Have No Common Intersections" << std::endl;
        
        message << street1_name << " and " << street2_name << " have no common intersections";
        app -> update_message(message.str());
    }
    
    else{
        std::cout << "Their Common Intersections are: " << std::endl;
        num_of_entry_per_line = 3;
        for(int comInts = 0; comInts < common_intersections.size(); comInts++){
            std::cout << comInts + 1 << ". " << getIntersectionName(common_intersections[comInts]) << "    ";
            
            num_of_entry_per_line--;
            if(num_of_entry_per_line == 0){
                num_of_entry_per_line = 3;
                std::cout << std::endl;
            }
            
            // Highlight common intersections
            intersections[common_intersections[comInts]].highlight = true;
        }
        
        if(num_of_entry_per_line != 3)
        std::cout << std::endl;
        
        message << "Common intersections of " << street1_name << " and " << street2_name << " are highlighted on the map";
        app -> update_message(message.str());
        
        // centre the map at the first common intersection
        centre_map_at_a_point(intersections[common_intersections[0]].xy_loc.x, intersections[common_intersections[0]].xy_loc.y, app);
    }
    app -> refresh_drawing();
}

void find_closest_POI_to_highlighted_intersection(GtkWidget* /*widget*/, ezgl::application* app){
    
    // Reset highlight status of POIs to false
    for(int pt_of_interest = 0; pt_of_interest < POIs.size(); pt_of_interest++)
        POIs[pt_of_interest].highlight = false;
    
    int num_intersection_highlighted = 0;
    int highlighted_intersection_id = -1;
    
    // Find and count highlighted intersections
    for(int intersection = 0; intersection < intersections.size(); intersection++){
        if(intersections[intersection].highlight){
            num_intersection_highlighted++;
            highlighted_intersection_id = intersection;
        }
    }
    
    std::stringstream message;
    
    // User need to highlight exactly one intersection that they want to find the closest POI with respect to
    if(num_intersection_highlighted != 1){
        std::cout << "Please highlight only one intersection that you want to find the closest POI with respect to and press the button again" << std::endl;
        message << "Please highlight only one intersection and press the button again";
        app -> update_message(message.str());
    }
    
    else{
        std::cout << "Point of Interest Types: " << std::endl;
        
        // List all POI types
        int num_of_entry_per_line = 6;
        for(int type = 0; type < POI_TYPES.size(); type++){
            std::cout << type + 1 << ". " << POI_TYPES[type] << "    ";
            num_of_entry_per_line--;
            if(num_of_entry_per_line == 0){
                num_of_entry_per_line = 6;
                std::cout << std::endl;
            }
        }
        
        if(num_of_entry_per_line != 0)
        std::cout << std::endl;
        
        // Ask user to select one
        int type_selected;
        std::string closest_POI_type;
        std::cout << "Please Enter the Number Corresponding to the Desired POI Type and then press <Enter>: ";
        std::cin >> type_selected;
        closest_POI_type = POI_TYPES[type_selected - 1];
        
        // Find the index of the closest POI of the desired type to the highlighted intersection and set its highlight status to true
        int closest_POI_id = findClosestPOI(getIntersectionPosition(highlighted_intersection_id), closest_POI_type);
        
        // If no POI found, the city does not have this type of POI
        if(closest_POI_id == -1){
            std::cout << "There is no " << closest_POI_type << " in this city" << std::endl;
            message << "There is no " << closest_POI_type << " in this city, " << "centering back to "
                    << intersections[highlighted_intersection_id].name;
            app -> update_message(message.str());
            
            // Centre the map at highlighted intersection
            centre_map_at_a_point(intersections[highlighted_intersection_id].xy_loc.x, 
                                  intersections[highlighted_intersection_id].xy_loc.y, app);
        }
        
        // Else, highlight that POI and centre the map at that POI
        else{
            
            // Set highlight status to true and display some info on command line and application window
            POIs[closest_POI_id].highlight = true;
            std::cout << "The closest " << closest_POI_type << " to " << intersections[highlighted_intersection_id].name <<
                      " is " << POIs[closest_POI_id].name << std::endl;
            
            message << "The closest " << closest_POI_type << " to " << intersections[highlighted_intersection_id].name <<
                      " is " << POIs[closest_POI_id].name;
            app -> update_message(message.str());
            
            // Centre the map at POI
            centre_map_at_a_point(POIs[closest_POI_id].xy_loc.x, POIs[closest_POI_id].xy_loc.y, app);
        }
        
        // Force redraw
        app -> refresh_drawing();
    }
}
