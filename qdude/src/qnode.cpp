/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qdude/qnode.hpp"
#include <QDebug>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qdude {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qdude");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("qt_chatter_topic", 1000);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qdude");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("qt_chatter_topic", 1000);

    // temp disable// sub = n.subscribe("dummy_talker_topic", 1000, &QNode::chatterCallback,this); //testing by nay//still in use


    sub = n.subscribe("dummy_talker_topic", 1000, &QNode::chatterCallback,this); //testing


    /*subscribe to image which is from RPi*/
    image_transport::ImageTransport imageTransport(n);
    image_subscriber = imageTransport.subscribe("/camera/image", 1,&QNode::imageCallback, this);


	start();
	return true;
}

/*this will be called back when msg arrives*/
void QNode::chatterCallback(const std_msgs::String::ConstPtr& msg)
{

  ROS_INFO("I can heard from : [%s]", msg->data.c_str());

}
/*this will be called back when image arrives*/
/*edited in CMakeLists.txt to use cv_bridge*/ //<- took me some time to figure it out
/*Reference from https://www.youtube.com/watch?v=W8LBtzDqYEs*/
void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg){

    cv_bridge::CvImagePtr cv_ptr;
      try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
        px = QPixmap::fromImage(cvtCvMat2QImage(cv_ptr->image));

    Q_EMIT updateImage(&px); //send back to main_window
}

/*once the key is pressed, announce that it is a valid data to send*/
void QNode::slotKeyPress(const QString key){
keyPressData = key;
isItValidToSendData = true;
}


void QNode::run() {
    ros::Rate loop_rate(10);
	int count = 0;
	while ( ros::ok() ) {

        /*only send once the key is activated*/
        if (isItValidToSendData == true) {
            isItValidToSendData = false;
            std_msgs::String msg;
            std::stringstream ss;
            ss << keyPressData.toStdString();
            msg.data = ss.str();
            chatter_publisher.publish(msg);
            log(Info,std::string("I sent: ")+msg.data);
            ++count;
        }

        ros::spinOnce();
        loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

/*Reference from https://www.youtube.com/watch?v=W8LBtzDqYEs*/
QImage QNode::cvtCvMat2QImage(const cv::Mat & image)
{
    QImage qtemp;
    if(!image.empty() && image.depth() == CV_8U)
    {
        const unsigned char * data = image.data;
        qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
        for(int y = 0; y < image.rows; ++y, data += image.cols*image.elemSize())
        {
            for(int x = 0; x < image.cols; ++x)
            {
                QRgb * p = ((QRgb*)qtemp.scanLine (y)) + x;
                *p = qRgb(data[x * image.channels()+2], data[x * image.channels()+1], data[x * image.channels()]);
            }
        }
    }
    else if(!image.empty() && image.depth() != CV_8U)
    {
        printf("Wrong image format, must be 8_bits\n");
    }
    return qtemp;
}














}  // namespace qdude
