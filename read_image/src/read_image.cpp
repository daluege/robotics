#include "read_image.h"
#include "std_msgs/Int16.h"
using namespace std;

#define PAINT_OUTPUT
#define READ_IMAGE_DEBUG
#define YAW_OFFSET -8
#define KP 0.5
#define KD 20
#define RATE 10

static const uint32_t MY_ROS_QUEUE_SIZE = 1;
std_msgs::Int16 steering;
float kp = KP, kd = KD;
int last_x = 0;
ros::Time last_time_twist;

cLaneDetectionFu::cLaneDetectionFu(ros::NodeHandle nh): nh_(nh), priv_nh_("~")
{
	std::string node_name = ros::this_node::getName();

	ROS_INFO("Node name: %s",node_name.c_str());

	//priv_nh_.param<std::string>(node_name + "/camera_name", camera_name, "/usb_cam/image_raw");
	priv_nh_.param<std::string>(node_name + "/camera_name", camera_name, "/app/camera/rgb/image_raw");
	priv_nh_.param<int>(node_name + "/cam_w", cam_w, 640);
	priv_nh_.param<int>(node_name + "/cam_h", cam_h, 480);
	read_images_ = nh.subscribe(nh_.resolveName(camera_name), MY_ROS_QUEUE_SIZE, &cLaneDetectionFu::ProcessInput,this);

	image_transport::ImageTransport image_transport(nh);
	image_publisher = image_transport.advertiseCamera("/lane_model/lane_model_image", MY_ROS_QUEUE_SIZE);
}

cLaneDetectionFu::~cLaneDetectionFu()
{
}

void cLaneDetectionFu::ProcessInput(const sensor_msgs::Image::ConstPtr& msg)
{
	//use ROS image_proc or opencv instead of ip mapper?
	cv_bridge::CvImagePtr cv_ptr;

	//convert to cvImagePtr and make it black and white (8bit)
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

	// matrix of the image
	cv::Mat image = cv_ptr->image.clone();

	// edge detection with OpenCV's Canny function
	cv::Mat canny_image;
	cv::Canny(image, canny_image, 50, 350);
	#ifdef PAINT_OUTPUT
	cv::imshow("black and white image", canny_image);
	cv::waitKey(1);
	#endif

    // Coordinate-related
	int desired_middle_point = canny_image.cols / 2;
	int middlepoint, x;
	int topleft, topright, bottomleft, bottomright;

    // Time-related
    ros::Time current_time_twist;
    double dx_dt;
    double dt_twist = (current_time_twist - last_time_twist).toSec();

	topleft = canny_image.cols;
	topright = 0;
	bottomleft = canny_image.cols;
	bottomright = 0;

	for (int j = 0; j < canny_image.cols; ++j) {
		int read_point = 0;
		// cv::Mat uses ROW-major system -> .at(y,x)
		// find leftmost and rightmost non black pixels at y-line 280
		read_point = canny_image.at<uint8_t>(280, j);
		if(read_point > 0) {
			if(topleft > j) topleft = j;
			if(topright < j) topright = j;
		}
	}
	for (int j = 0; j < canny_image.cols; ++j) {
		int read_point = 0;
		//cv::Mat uses ROW-major system -> .at(y,x)
		// find leftmost and rightmost non black pixels at y-line 350
		read_point = canny_image.at<uint8_t>(350, j);
		if(read_point > 0){;
		   	if(bottomleft > j) bottomleft = j;
			if(bottomright < j) bottomright = j;				 
		}
	}
	//ROS_INFO("bottomleft: %i , bottomright: %i , topleft: %i , topright: %i" , bottomleft, bottomright, topleft, topright);
	middlepoint = (bottomleft + bottomright + topleft + topright ) / 4 ;

	x = desired_middle_point - middlepoint;
    current_time_twist = ros::Time::now();
    dx_dt = ((double) (x - last_x)) / 100; // dt_twist;

	// simple P-controler
	steering.data = -(int(kp * (-x)) + int(kd * (-dx_dt))) + 90 + YAW_OFFSET;
	// limit steering data (0..180)
	if(steering.data < 0) steering.data = 0;
	if(steering.data > 180) steering.data = 180;
	#ifdef READ_IMAGE_DEBUG
	ROS_INFO("x is : %i, change rate: %f, change Steering Data is: %i", x, dx_dt, steering.data);
	#endif

    last_x = x;
    last_time_twist = current_time_twist;
}

void cLaneDetectionFu::pubRGBImageMsg(cv::Mat& rgb_mat)
{
    sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

    ros::Time head_time_stamp = ros::Time::now();
    std::string rgb_frame_id = "_rgb_optical_frame";
    sensor_msgs::CameraInfoPtr rgb_camera_info;
    unsigned int head_sequence_id = 0;

    rgb_img->header.seq = head_sequence_id;
    rgb_img->header.stamp = head_time_stamp;
    rgb_img->header.frame_id = rgb_frame_id;

    rgb_img->width = rgb_mat.cols;
    rgb_img->height = rgb_mat.rows;

    rgb_img->encoding = sensor_msgs::image_encodings::BGR8;
    rgb_img->is_bigendian = 0;

    int step = sizeof(unsigned char) * 3 * rgb_img->width;
    int size = step * rgb_img->height;
    rgb_img->step = step;
    rgb_img->data.resize(size);
    memcpy(&(rgb_img->data[0]), rgb_mat.data, size);

    rgb_camera_info->header.frame_id = rgb_frame_id;
    rgb_camera_info->header.stamp = head_time_stamp;
    rgb_camera_info->header.seq = head_sequence_id;

    image_publisher.publish(rgb_img, rgb_camera_info);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "cLaneDetectionFu");
	ros::NodeHandle nh,sh;
	ros:: Publisher pubSteering = sh.advertise<std_msgs::Int16>(sh.resolveName("manual_control/steering"),10,false);
	
	cLaneDetectionFu node=cLaneDetectionFu(nh);

	ros::Rate r(RATE); // Rate: loops per second
	while(ros::ok())
	{
		pubSteering.publish(steering);
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
