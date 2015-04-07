/**
 * @file /include/qdude/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qdude_QNODE_HPP_
#define qdude_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QObject>
#include <QStringListModel>

#include <std_msgs/String.h> //ros//required for string mag//chatter

//for image
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qdude {

/*****************************************************************************
** Class
*****************************************************************************/

//class QNode : public QThread  {
class QNode : public QThread  {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
        void chatterCallback(const std_msgs::String::ConstPtr &msg); //testing by nay
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        void imageCb(const sensor_msgs::ImageConstPtr& msg);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

 Q_SIGNALS:
        //signals:
	void loggingUpdated();
        void rosShutdown();
        void updateImage(QPixmap *px);

public Q_SLOTS:
        void slotKeyPress(const QString key);

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
	image_transport::Subscriber image_subscriber;
        ros::Subscriber sub;
        QStringListModel logging_model;
        bool isItValidToSendData;
        QString keyPressData;
        QImage cvtCvMat2QImage(const cv::Mat & image);
        QPixmap px;
};

}  // namespace qdude

#endif /* qdude_QNODE_HPP_ */
