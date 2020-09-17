//
// Created by maxandre on 17.09.20.
//

#ifndef VEHICLE_PLATFORM_RADAR_TOPIC_MERGER_H
#define VEHICLE_PLATFORM_RADAR_TOPIC_MERGER_H

#include <ros/subscriber.h>
#include <delphi_esr_msgs/EsrTrack.h>
#include <delphi_esr_msgs/EsrTrackMotionPower.h>
#include <string>
#include <vehicle_platform/EsrTrackArray.h>

using namespace std;

class radar_topic_merger {
    public:
    explicit radar_topic_merger(ros::NodeHandle* nodehandle);

    void esrtrack_callback(const delphi_esr_msgs::EsrTrackConstPtr &esrtrack);
    void trackmotionpower_callback(const delphi_esr_msgs::EsrTrackMotionPower &trackmotionpower);

    private:
    ros::NodeHandle nh;
    vehicle_platform::EsrTrackArray esrTrackArray;
    int endOfScanSeq = 0;

    ros::Subscriber esrTrackSub;

    ros::Publisher esrTrackArrayPub;

    string esrTrackTopic;
};


#endif //VEHICLE_PLATFORM_RADAR_TOPIC_MERGER_H
