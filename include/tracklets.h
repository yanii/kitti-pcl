#pragma once

#include <string>
#include <vector>
#include <fstream>

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

class Tracklets {

public:
  
  // pose states
  enum POSE_STATES {
    UNSET   = 0,
    INTERP  = 1,
    LABELED = 2
  };

  // occlusion states
  enum OCCLUSION_STATES {
    OCCLUSION_UNSET = -1,
    VISIBLE         = 0,
    PARTLY          = 1,
    FULLY           = 2
  };

  // occlusion states
  enum TRUNCATION_STATES {
    TRUNCATION_UNSET  = -1,
    IN_IMAGE          = 0,
    TRUNCATED         = 1,
    OUT_IMAGE         = 2,
    BEHIND_IMAGE      = 99
  };

  // constructor / deconstructor
  Tracklets () {}
  ~Tracklets () {}

  // pose of tracklet at specific frame
  struct tPose{

    double            tx,ty,tz;         // translation wrt. Velodyne coordinates
    double            rx,ry,rz;         // rotation wrt. Velodyne coordinates
    POSE_STATES       state;            // pose state
    OCCLUSION_STATES  occlusion;        // occusion state
    bool              occlusion_kf;     // is this an occlusion keyframe
    TRUNCATION_STATES truncation;       // truncation state
    float             amt_occlusion;    // Mechanical Turk occlusion label
    float             amt_border_l;     // Mechanical Turk left boundary label (relative)
    float             amt_border_r;     // Mechanical Turk right boundary label (relative)
    int               amt_occlusion_kf; // Mechanical Turk occlusion keyframe
    int               amt_border_kf;    // Mechanical Turk border keyframe

    tPose() {}
    tPose(double tx, double ty, double tz, double rx, double ry, double rz,
          POSE_STATES state, OCCLUSION_STATES occlusion, TRUNCATION_STATES truncation)
      : tx(tx), ty(ty), tz(tz), rx(rx), ry(ry), rz(rz), state(state), 
        occlusion(occlusion), occlusion_kf(false), truncation(truncation)/*, geometry()*/ {}

      template<typename Archive> void load(Archive &ar, const unsigned int version){
        ar & BOOST_SERIALIZATION_NVP(tx)
           & BOOST_SERIALIZATION_NVP(ty)
           & BOOST_SERIALIZATION_NVP(tz)
           & BOOST_SERIALIZATION_NVP(rx)
           & BOOST_SERIALIZATION_NVP(ry)
           & BOOST_SERIALIZATION_NVP(rz)
           & BOOST_SERIALIZATION_NVP(state);

        // version 1
        if (version > 0){
          ar & BOOST_SERIALIZATION_NVP(occlusion)
             & BOOST_SERIALIZATION_NVP(occlusion_kf)
             & BOOST_SERIALIZATION_NVP(truncation);

        // default values
        } else {
          occlusion    = OCCLUSION_UNSET;
          occlusion_kf = false;
          truncation   = IN_IMAGE;
        }

        // version 2
        if(version > 1){
          ar & BOOST_SERIALIZATION_NVP(amt_occlusion)
             & BOOST_SERIALIZATION_NVP(amt_occlusion_kf)
             & BOOST_SERIALIZATION_NVP(amt_border_l)
             & BOOST_SERIALIZATION_NVP(amt_border_r)
             & BOOST_SERIALIZATION_NVP(amt_border_kf);

        // default values
        } else{
          amt_occlusion    = -1;
          amt_occlusion_kf = -1;
          amt_border_l     = -1;
          amt_border_r     = -1;
          amt_border_kf    = -1;
        }
      }

    template<typename Archive> void save(Archive &ar, const unsigned int) const{
      ar & BOOST_SERIALIZATION_NVP(tx)
         & BOOST_SERIALIZATION_NVP(ty)
         & BOOST_SERIALIZATION_NVP(tz)
         & BOOST_SERIALIZATION_NVP(rx)
         & BOOST_SERIALIZATION_NVP(ry)
         & BOOST_SERIALIZATION_NVP(rz)
         & BOOST_SERIALIZATION_NVP(state)
         & BOOST_SERIALIZATION_NVP(occlusion)
         & BOOST_SERIALIZATION_NVP(occlusion_kf)
         & BOOST_SERIALIZATION_NVP(truncation)
         & BOOST_SERIALIZATION_NVP(amt_occlusion)
         & BOOST_SERIALIZATION_NVP(amt_occlusion_kf)
         & BOOST_SERIALIZATION_NVP(amt_border_l)
         & BOOST_SERIALIZATION_NVP(amt_border_r)
         & BOOST_SERIALIZATION_NVP(amt_border_kf);
    }

    // for versioning
    BOOST_SERIALIZATION_SPLIT_MEMBER()
  };

  // tracklet with meta information and vector of poses
  struct tTracklet {

    std::string        objectType;  // object type: 'Car', 'Pedestrian', etc.
    float              h,w,l;       // height, width, length of bounding box
    int                first_frame; // number of first frame of tracklet
    std::vector<tPose> poses;       // poses of this tracklet
    int                finished;    // is this tracklet fully labeled?

    tTracklet(){}
    tTracklet(std::string objectType, float h, float w, float l, int first_frame, std::vector<tPose> poses, int finished)
      : objectType(objectType), h(h), w(w), l(l), first_frame(first_frame), poses(poses), finished(finished) {}

    template<typename Archive> void serialize(Archive &ar, const unsigned int version){
      ar & BOOST_SERIALIZATION_NVP(objectType)
         & BOOST_SERIALIZATION_NVP(h)
         & BOOST_SERIALIZATION_NVP(w)
         & BOOST_SERIALIZATION_NVP(l)
         & BOOST_SERIALIZATION_NVP(first_frame)
         & BOOST_SERIALIZATION_NVP(poses);
      if(version > 0) ar & BOOST_SERIALIZATION_NVP(finished);
      else            finished = 0;
    }

    // return last frame index of tracklet wrt. to sequence
    int lastFrame() {
      return first_frame+poses.size()-1;
    }
  };
  
  // total number of tracklets
  int  numberOfTracklets() {
    return tracklets.size();
  }
  
  // get tracklet with given id, ranging [0..#tracklets-1]
  tTracklet* getTracklet (int tracklet_id) {
    return &tracklets[tracklet_id];
  }
  
  // push back a tracklet
  void addTracklet (tTracklet tracklet) {
    tracklets.push_back(tracklet);
  }
  
  // returns pose pointer, if requested tracklet is active at given frame
  bool getPose (int tracklet_id, int frame_number,tPose* &pose) {
    if (!isActive(tracklet_id,frame_number)) {
      return false;
    } else {
      int pose_idx = frame_number-tracklets[tracklet_id].first_frame;
      pose = &(tracklets[tracklet_id].poses[pose_idx]);
      return true;
    }
  }
  
  // checks if tracklet wit given id exists at given frame
  bool isActive (int tracklet_id, int frame_number) {
    if (tracklet_id<0 || tracklet_id>=(int)tracklets.size())
      return false;
    int pose_idx = frame_number-tracklets[tracklet_id].first_frame;
    if (pose_idx<0 || pose_idx>=(int)tracklets[tracklet_id].poses.size())
      return false;
    return true;
  }
  
  // load tracklets from xml file
  bool loadFromFile (std::string filename) {
    try {
      std::ifstream ifs(filename.c_str());
      boost::archive::xml_iarchive ia(ifs);
      ia >> boost::serialization::make_nvp("tracklets", tracklets);
      return true;
    } catch (...) {
      return false;
    }
  }
  
  // save tracklets to xml file, try several times
  bool saveToFile (std::string filename) {
    for (int i=0; i<10; i++) {
      try {
        std::ofstream ofs(filename.c_str());
        boost::archive::xml_oarchive oa(ofs);
        oa << boost::serialization::make_nvp("tracklets", tracklets);
        return true;
      } catch(...) {
        usleep(200000);
      }
    }
    return false;
  }
  
private:

  std::vector<tTracklet> tracklets;

};

// set the version of the stored tracklet data
BOOST_CLASS_VERSION(Tracklets::tTracklet, 1)
BOOST_CLASS_VERSION(Tracklets::tPose, 2)

