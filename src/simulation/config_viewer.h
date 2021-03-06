#ifndef _CONFIG_VIEWER_H_
#define _CONFIG_VIEWER_H_

#include "utils/config.h"
#include <btBulletDynamicsCommon.h>

struct ViewerConfig : Config {
    static btVector3 cameraHomePosition;
    static int windowWidth;
    static int windowHeight;
  ViewerConfig() : Config() {
    params.push_back(new Parameter<int>("windowWidth", &windowWidth, "viewer window width"));
    params.push_back(new Parameter<int>("windowHeight", &windowHeight, "viewer window height"));
  }
};

#endif // _CONFIG_VIEWER_H_
