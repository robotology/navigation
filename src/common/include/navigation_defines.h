/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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
    #define TF_REMOTE_PARAM_NAME                  "ft_server_prefix"
    #define TF_LOCAL_PARAM_NAME                   "ft_client_prefix"
    #define TF_CLIENT_DEFAULT_CONFIG              "ftc_yarp_only.xml"
#else
    #define LOCALIZATION_CLIENT_DEVICE_DEFAULT    "localization2DClient"
    #define MAP_CLIENT_DEVICE_DEFAULT             "map2DClient"
    #define NAVIGATION_CLIENT_DEVICE_DEFAULT      "navigation2DClient"
    #define LIDAR_CLIENT_DEVICE_DEFAULT           "Rangefinder2DClient"
    #define TF_CLIENT_DEFAULT_DEVICE              "transformClient"
    #define MAP_REMOTE_PORT_DEFAULT               "/mapServer"
    #define NAVIGATION_REMOTE_PORT_DEFAULT        "/navigationServer"
    #define LOCALIZATION_REMOTE_PORT_DEFAULT      "/localizationServer"
    #define TF_REMOTE_PARAM_NAME                  "remote"
    #define TF_LOCAL_PARAM_NAME                   "local"
#endif
