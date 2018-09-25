# navigationTemplate
 A skeleton application which can be used as a template to develop a new navigation module.
 The user has to complete the parts marked with the comment //### TO BE IMPLEMENTED BY USER in order to implement the navigation functionalities.
 The method *navigationModule::respond()* implements a default parser for the basic rpc commands needed to communicate with a Navigation2DClient. 
 This is the list of default rpc commands:
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GOTOABS] <x> <y> <angle in degrees>** Starts a navigation task to reach a goal defined in the map reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GOTOREL]  <x> <y> <angle in degrees>** Starts a navigation task to reach a goal defined in the robot reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_STATUS]** Returns the current navigation status, expressed as a yarp::dev::INavigation2D::NavigationStatusEnum.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_STOP]** Stops the current navigation task. It is also used to clear a previouly aborted navigation task.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_SUSPEND]** Pauses the current navigation task until a resume command is issued. 
* **[VOCAB_INAVIGATION] [VOCAB_NAV_RESUME]** Resumes the current navigation params.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_CURRENT_POS]** Returns the current robot position in map reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_ABS_TARGET]** Returns the current navigation target expressed in the map reference frame.
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_REL_TARGET]** Returns the current navigation target expressed in the robot reference frame.
