#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

using namespace message_filters;

typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ImageSyncPolicy;

class ImageSync 
{
public:
    ImageSync(ros::NodeHandle& nh) {
        kinect1_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/kinect1/rgb/image_color", 5);
        kinect2_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/kinect2/hd/image_color", 5);
        astra_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/astra_camera/color/image_raw", 5);

        sync = new Synchronizer<ImageSyncPolicy>(ImageSyncPolicy(10), *kinect1_sub, *kinect2_sub, *astra_sub);
        sync->registerCallback(boost::bind(&ImageSync::syncCallback, this, _1, _2, _3));

        kinect1_pub = nh.advertise<sensor_msgs::Image>("/kinect1/rgb/image_color_sync", 5);
        kinect2_pub = nh.advertise<sensor_msgs::Image>("/kinect2/hd/image_color_sync", 5);
        astra_pub = nh.advertise<sensor_msgs::Image>("/astra_camera/color/image_raw_sync", 5);
        
    }

    ~ImageSync() {
        delete kinect1_sub;
        delete kinect2_sub;
        delete astra_sub;
        delete sync;
    }

    void syncCallback(const sensor_msgs::ImageConstPtr& kinect1_image, const sensor_msgs::ImageConstPtr& kinect2_image, const sensor_msgs::ImageConstPtr& astra_image) {
        kinect1_pub.publish(kinect1_image);
        kinect2_pub.publish(kinect2_image);
        astra_pub.publish(astra_image);
    }

private:
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> *kinect1_sub;
    message_filters::Subscriber<sensor_msgs::Image> *kinect2_sub;
    message_filters::Subscriber<sensor_msgs::Image> *astra_sub;
    
    ros::Publisher kinect1_pub;
    ros::Publisher kinect2_pub;
    ros::Publisher astra_pub;
    
    Synchronizer<ImageSyncPolicy> *sync;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_sync");
    ros::NodeHandle nh;

    ImageSync image_sync(nh);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::waitForShutdown();
}