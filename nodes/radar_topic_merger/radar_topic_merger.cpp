//
// Created by maxandre on 17.09.20.
//
#include <ros/ros.h>
#include <ros/console.h>
#include <delphi_esr_msgs/EsrTrack.h>
#include <delphi_esr_msgs/EsrTrackMotionPower.h>
#include <vehicle_platform/EsrTrackArray.h>
#include <vehicle_platform/EsrTrackMotionPowerArray.h>
#include <vehicle_platform/EsrTrackMotionPowerId.h>
#include "radar_topic_merger.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "radar_topic_merger");
    ros::NodeHandle nh("~");

    radar_topic_merger radar_topic_merger(&nh);

    ros::spin();
    return 0;
}

radar_topic_merger::radar_topic_merger(ros::NodeHandle *nodehandle) : nh(*nodehandle) {
    this->nh.param<string>("esrtrack_topic", this->esrTrackTopic, "/radar_fc/parsed_tx/radartrack");
    this->esrTrackSub = this->nh.subscribe(this->esrTrackTopic, 10, &radar_topic_merger::esrtrack_callback, this);
    this->esrTrackArrayPub = nh.advertise<vehicle_platform::EsrTrackArray>("/radar_fc/merged/radartracks", 10);

    this->nh.param<string>("trackpower_topic", this->trackPowerTopic, "/radar_fc/parsed_tx/trackmotionpower");
    this->trackPowerSub = this->nh.subscribe(this->trackPowerTopic, 10, &radar_topic_merger::trackpower_callback, this);
    this->trackPowerPub = nh.advertise<vehicle_platform::EsrTrackMotionPowerArray>("/radar_fc/merged/trackmotionpower", 10);

    ROS_INFO_STREAM(typeid(*this).name() << " - Node Started");
}

void radar_topic_merger::trackpower_callback(const delphi_esr_msgs::EsrTrackMotionPower &trackPower) {
    bool skip = false;
    if (trackPower.header.seq < this->endOfScanPowerSeq) { /* We make sure we only keep current scan messages */
        ROS_WARN_STREAM(typeid(*this).name() << " - Receiving trackPower data in wrong order");
        skip = true;
    }

    unsigned short int offset = (unsigned short int)trackPower.group_id * 7;
    // Last trackPower only contains one trackpower, so if offset+i >= 64 we skip so we don't try to access tracks that doesn't exist.
    for (int i = 0; i < 7 && offset+i < 64 && !skip; i++) {
        vehicle_platform::EsrTrackMotionPowerId trackPowerWithId;
        trackPowerWithId.power = trackPower.tracks[offset+i].power;
        trackPowerWithId.movable_fast = trackPower.tracks[offset+i].movable_fast;
        trackPowerWithId.movable_slow = trackPower.tracks[offset+i].movable_slow;
        trackPowerWithId.moving = trackPower.tracks[offset+i].moving;
        trackPowerWithId.track_id = offset+i;
        this->trackPowerArray.trackpower.push_back(trackPowerWithId);
    }

    // If not last message of the scan we exit.
    if (trackPower.group_id != 9) return;

    this->endOfScanPowerSeq = trackPower.header.seq;
    this->trackPowerArray.header.stamp = trackPower.header.stamp;
    this->trackPowerPub.publish(this->trackPowerArray);
    this->trackPowerArray.trackpower.clear();
}

void radar_topic_merger::esrtrack_callback(const delphi_esr_msgs::EsrTrackConstPtr &esrTrack) {
    bool skip = false;
    if (!this->esrTrackArray.EsrTracks.empty()
        && esrTrack->header.seq < this->esrTrackArray.EsrTracks.back().header.seq) { // We make sure messages are coming in the right order
        ROS_WARN_STREAM(typeid(*this).name() << " - Receiving esrTrack data in wrong order");
        skip = true;
    }

    /* We skip empty tracks */
    if ((int)esrTrack->track_status == 0)  {
        skip = true;
    }

    if (esrTrack->header.seq < this->endOfScanTrackSeq) { /* We make sure we only keep current scan messages */
        ROS_WARN_STREAM(typeid(*this).name() << " - Receiving esrTrack data from previous scans");
        skip = true;
    }

    if (!skip) {
        this->esrTrackArray.EsrTracks.push_back(*esrTrack);
    }
    // Nothing more to do if we haven't reached the end of the array.
    if (esrTrack->track_id < 63) return;

    this->endOfScanTrackSeq = esrTrack->header.seq;

    // We set the timestamp of the array msg to the timestamp of the last element to allow sync.
    this->esrTrackArray.header.stamp = esrTrack->header.stamp;

    this->esrTrackArrayPub.publish(this->esrTrackArray);
    this->esrTrackArray.EsrTracks.clear();
}