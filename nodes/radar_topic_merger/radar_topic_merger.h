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
    void trackpower_callback(const delphi_esr_msgs::EsrTrackMotionPower &trackPower);

    private:
    ros::NodeHandle nh;

    vehicle_platform::EsrTrackArray esrTrackArray;
    vehicle_platform::EsrTrackMotionPowerArray trackPowerArray;

    unsigned int endOfScanTrackSeq = 0;
    unsigned int endOfScanPowerSeq = 0;

    ros::Subscriber esrTrackSub;
    ros::Subscriber trackPowerSub;

    ros::Publisher esrTrackArrayPub;
    ros::Publisher trackPowerPub;

    string esrTrackTopic;
    string trackPowerTopic;
};


#endif //VEHICLE_PLATFORM_RADAR_TOPIC_MERGER_H
