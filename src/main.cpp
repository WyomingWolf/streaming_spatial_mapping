///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*************************************************************************
 ** This sample shows how to capture a real-time 3D reconstruction      **
 ** of the scene using the Spatial Mapping API. The resulting mesh      **
 ** is displayed as a wireframe on top of the left image using OpenGL.  **
 ** Spatial Mapping can be started and stopped with the space bar key   **
 *************************************************************************/

 // Standard includes
#include <stdio.h>
#include <direct.h>
#include <string.h>

 // ZED includes
#include <sl/Camera.hpp>

 // Sample includes
#include "GLViewer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;

// Sample functions
void switchCameraSettings();
void switchViewMode();

// Sample variables
VIDEO_SETTINGS camera_settings_ = VIDEO_SETTINGS::BRIGHTNESS;
VIEW view_mode = VIEW::LEFT;
string str_camera_settings = "BRIGHTNESS";
int step_camera_setting = 1;
bool led_on = true;

// set to 0 to create a Fused Point Cloud
#define CREATE_MESH 1

void parseArgs(int argc, char **argv,sl::InitParameters& param);

vector< string> split(const string& s, char seperator) {
	vector< string> output;
	string::size_type prev_pos = 0, pos = 0;

	while ((pos = s.find(seperator, pos)) != string::npos) {
		string substring(s.substr(prev_pos, pos - prev_pos));
		output.push_back(substring);
		prev_pos = ++pos;
	}

	output.push_back(s.substr(prev_pos, pos - prev_pos));
	return output;
}

void setStreamParameter(InitParameters& init_p, string& argument) {
	vector< string> configStream = split(argument, ':');
	String ip(configStream.at(0).c_str());
	if (configStream.size() == 2) {
		init_p.input.setFromStream(ip, atoi(configStream.at(1).c_str()));
	}
	else init_p.input.setFromStream(ip);
}

int main(int argc, char** argv) {
#if 0
	auto streaming_devices = Camera::getStreamingDeviceList();
	int nb_streaming_zed = streaming_devices.size();

	print("Detect: " + to_string(nb_streaming_zed) + " ZED in streaming");
	if (nb_streaming_zed == 0) {
		print("No streaming ZED detected, have you take a look to the sample 'ZED Streaming Sender' ?");
		return 0;
	}

	for (auto& it : streaming_devices)
		cout << "* ZED: " << it.serial_number << ", IP: " << it.ip << ", port : " << it.port << ", bitrate : " << it.current_bitrate << "\n";
#endif

	Camera zed;
	// Set configuration parameters for the ZED
	InitParameters init_parameters;
	//init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE;
	init_parameters.coordinate_units = UNIT::METER;
	init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
	init_parameters.sdk_verbose = true;
	parseArgs(argc, argv, init_parameters);

	string stream_params;
	if (argc > 1) {
		stream_params = string(argv[1]);
	}
	else {
		//cout << "\nOpening the stream requires the IP of the sender\n";
		//cout << "Usage : ./ZED_Streaming_Receiver IP:[port]\n";
		//cout << "You can specify it now, then press ENTER, 'IP:[port]': ";
		//cin >> stream_params;
        stream_params = "alpha";
	}

	setStreamParameter(init_parameters, stream_params);

	// Open the camera
	auto returned_state = zed.open(init_parameters);
	if (returned_state != ERROR_CODE::SUCCESS) {
		print("Camera Open", returned_state, "Exit program.");
		return EXIT_FAILURE;
	}

	// Print camera information
	auto camera_info = zed.getCameraInformation();
	cout << endl;
	cout << "ZED Model                 : " << camera_info.camera_model << endl;
	cout << "ZED Serial Number         : " << camera_info.serial_number << endl;
	cout << "ZED Camera Firmware       : " << camera_info.camera_configuration.firmware_version << "/" << camera_info.sensors_configuration.firmware_version << endl;
	cout << "ZED Camera Resolution     : " << camera_info.camera_configuration.resolution.width << "x" << camera_info.camera_configuration.resolution.height << endl;
	cout << "ZED Camera FPS            : " << zed.getInitParameters().camera_fps << endl;

	// Initialise camera setting
	switchCameraSettings();

#if CREATE_MESH
    Mesh map; // Current incremental mesh
#else
    FusedPointCloud map; // Current incremental fused point cloud
#endif

    CameraParameters camera_parameters = zed.getCameraInformation().camera_configuration.calibration_parameters.left_cam;

    GLViewer viewer;
    bool error_viewer = viewer.init(argc, argv, camera_parameters, &map);

    if(error_viewer) {
        viewer.exit();
        zed.close();
        return EXIT_FAILURE;
    }

    Mat image; // Current left image
    Pose pose; // Camera pose tracking data

    SpatialMappingParameters spatial_mapping_parameters;
    POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;
    SPATIAL_MAPPING_STATE mapping_state = SPATIAL_MAPPING_STATE::NOT_ENABLED;
    bool mapping_activated = false; // Indicates if spatial mapping is running or not
    chrono::high_resolution_clock::time_point ts_last; // Timestamp of the last mesh request
    
    // Enable positional tracking before starting spatial mapping
    returned_state = zed.enablePositionalTracking();
    if(returned_state != ERROR_CODE::SUCCESS) {
        print("Enabling positional tracking failed: ", returned_state);
        zed.close();
        return EXIT_FAILURE;
    }

    while(viewer.isAvailable()) {
        if(zed.grab() == ERROR_CODE::SUCCESS) {
            // Retrieve image in GPU memory
            zed.retrieveImage(image, VIEW::LEFT, MEM::GPU);
            // Update pose data (used for projection of the mesh over the current image)
            tracking_state = zed.getPosition(pose);

            if(mapping_activated) {
                mapping_state = zed.getSpatialMappingState();
                // Compute elapsed time since the last call of Camera::requestSpatialMapAsync()
                auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();
                // Ask for a mesh update if 500ms elapsed since last request
                if((duration > 250) && viewer.chunksUpdated()) {
                    zed.requestSpatialMapAsync();
                    ts_last = chrono::high_resolution_clock::now();
                }

                if(zed.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS) {
                    zed.retrieveSpatialMapAsync(map);
                    viewer.updateChunks();
                }
            }

            bool change_state = viewer.updateImageAndState(image, pose.pose_data, tracking_state, mapping_state);

            if(change_state) {
                if(!mapping_activated) {
                    Transform init_pose;
                    zed.resetPositionalTracking(init_pose);

                    // Configure Spatial Mapping parameters
					SpatialMappingParameters spatial_mapping_parameters;
					//spatial_mapping_parameters.resolution_meter = SpatialMappingParameters::get(SpatialMappingParameters::MAPPING_RESOLUTION::MEDIUM);
					spatial_mapping_parameters.resolution_meter = 0.150;
					spatial_mapping_parameters.range_meter = 12;
                    spatial_mapping_parameters.use_chunk_only = true;
                    spatial_mapping_parameters.save_texture = false;
					// cout << "Initalized Spatial Mapping Paramters" << endl;
#if CREATE_MESH
					spatial_mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
#else
					spatial_mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
#endif					
                    // Enable spatial mapping
                    try {
                        zed.enableSpatialMapping(spatial_mapping_parameters);
                        print("Spatial Mapping will output a " + string(toString(spatial_mapping_parameters.map_type).c_str()));
                    } catch(string e) {
                        print("Error enable Spatial Mapping "+ e);
                    }

                    // Clear previous Mesh data
                    map.clear();
                    viewer.clearCurrentMesh();

                    // Start a timer, we retrieve the mesh every XXms.
                    ts_last = chrono::high_resolution_clock::now();

                    mapping_activated = true;
                } else {
                    // Extract the whole mesh
                    zed.extractWholeSpatialMap(map);
#if CREATE_MESH
                    MeshFilterParameters filter_params;
                    filter_params.set(MeshFilterParameters::MESH_FILTER::MEDIUM);
                    // Filter the extracted mesh
                    map.filter(filter_params, true);
					viewer.clearCurrentMesh();

                    // If textures have been saved during spatial mapping, apply them to the mesh
                    if(spatial_mapping_parameters.save_texture)
                        map.applyTexture(MESH_TEXTURE_FORMAT::RGB);
#endif
                    // Save mesh as an OBJ file
                    string cwd = _getcwd(NULL, 0);
                    string saveName = cwd + "\\mesh_gen.obj";
                    bool error_save = map.save(saveName.c_str());
                    if(error_save)
                        print("Mesh saved under: " +saveName);
					else
                        print("Failed to save the mesh under: " +saveName);

                    mapping_state = SPATIAL_MAPPING_STATE::NOT_ENABLED;
                    mapping_activated = false;
                }
            }
        }
    }

    image.free();
    map.clear();

    zed.disableSpatialMapping();
    zed.disablePositionalTracking();
    zed.close();
    return EXIT_SUCCESS;
}

/**
	This function toggles between camera settings
 **/
void switchCameraSettings() {
	camera_settings_ = static_cast<VIDEO_SETTINGS> ((int)camera_settings_ + 1);

	// reset to 1st setting
	if (camera_settings_ == VIDEO_SETTINGS::LED_STATUS)
		camera_settings_ = VIDEO_SETTINGS::BRIGHTNESS;

	// select the right step
	step_camera_setting = (camera_settings_ == VIDEO_SETTINGS::WHITEBALANCE_TEMPERATURE) ? 100 : 1;

	// get the name of the selected SETTING
	str_camera_settings = string(sl::toString(camera_settings_).c_str());

	print("Switch to camera settings: ", ERROR_CODE::SUCCESS, str_camera_settings);
}

/**
	This function toggles between view mode
 **/
void switchViewMode() {
	view_mode = static_cast<VIEW> ((int)view_mode + 1);

	// reset to 1st setting
	if (view_mode == VIEW::DEPTH_RIGHT)
		view_mode = VIEW::LEFT;


	print("Switch to view mode: ", ERROR_CODE::SUCCESS, string(sl::toString(view_mode).c_str()));
}

void parseArgs(int argc, char **argv,sl::InitParameters& param)
{
    if (argc > 1 && string(argv[1]).find(".svo")!=string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        cout<<"[Sample] Using SVO File input: "<<argv[1]<<endl;
    } else if (argc > 1 && string(argv[1]).find(".svo")==string::npos) {
        string arg = string(argv[1]);
        unsigned int a,b,c,d,port;
        if (sscanf(arg.c_str(),"%u.%u.%u.%u:%d", &a, &b, &c, &d,&port) == 5) {
            // Stream input mode - IP + port
            string ip_adress = to_string(a)+"."+to_string(b)+"."+to_string(c)+"."+to_string(d);
            param.input.setFromStream(sl::String(ip_adress.c_str()),port);
            cout<<"[Sample] Using Stream input, IP : "<<ip_adress<<", port : "<<port<<endl;
        }
        else  if (sscanf(arg.c_str(),"%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(sl::String(argv[1]));
            cout<<"[Sample] Using Stream input, IP : "<<argv[1]<<endl;
        }
        else if (arg.find("HD2K")!=string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD2K;
            cout<<"[Sample] Using Camera in resolution HD2K"<<endl;
        } else if (arg.find("HD1080")!=string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD1080;
            cout<<"[Sample] Using Camera in resolution HD1080"<<endl;
        } else if (arg.find("HD720")!=string::npos) {
            param.camera_resolution = sl::RESOLUTION::HD720;
            cout<<"[Sample] Using Camera in resolution HD720"<<endl;
        } else if (arg.find("VGA")!=string::npos) {
            param.camera_resolution = sl::RESOLUTION::VGA;
            cout<<"[Sample] Using Camera in resolution VGA"<<endl;
        }
    } else {
        // Default
    }
}
