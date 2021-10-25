/*
 * Copyright (C) 2006-2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */
 
#define BASECONTROL_COMMAND_VELOCIY_CARTESIAN 3
#define BASECONTROL_COMMAND_VELOCIY_POLAR     2
#define BASECONTROL_COMMAND_PERCENT_POLAR     1


#ifdef NAVIGATION_USE_NWC
    #define LOCALIZATION_CLIENT_DEVICE_DEFAULT    "localization2D_nwc_yarp"
    #define MAP_CLIENT_DEVICE_DEFAULT             "map2D_nwc_yarp"
    #define NAVIGATION_CLIENT_DEVICE_DEFAULT      "navigation2D_nwc_yarp"
    //#define LIDAR_CLIENT_DEVICE_DEFAULT           "rangefinder2D_nwc_yarp"
      #define LIDAR_CLIENT_DEVICE_DEFAULT           "Rangefinder2DClient"
    #define TF_CLIENT_DEFAULT_DEVICE              "frameTransformClient"
    #define MAP_REMOTE_PORT_DEFAULT               "/map2D_nws_yarp"
    #define NAVIGATION_REMOTE_PORT_DEFAULT        "/navigation2D_nws_yarp"
    #define LOCALIZATION_REMOTE_PORT_DEFAULT      "/localization2D_nws_yarp"
#else
    #define LOCALIZATION_CLIENT_DEVICE_DEFAULT    "localization2DClient"
    #define MAP_CLIENT_DEVICE_DEFAULT             "map2DClient"
    #define NAVIGATION_CLIENT_DEVICE_DEFAULT      "navigation2DClient"
    #define LIDAR_CLIENT_DEVICE_DEFAULT           "Rangefinder2DClient"
    #define TF_CLIENT_DEFAULT_DEVICE              "transformClient"
    #define MAP_REMOTE_PORT_DEFAULT               "/mapServer"
    #define NAVIGATION_REMOTE_PORT_DEFAULT        "/navigationServer"
    #define LOCALIZATION_REMOTE_PORT_DEFAULT      "/localizationServer"
#endif
