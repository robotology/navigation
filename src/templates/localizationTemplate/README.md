# localizationTemplate
 A skeleton application which can be used as a template to develop a new localization module.
 The user has to complete the parts marked with the comment //### TO BE IMPLEMENTED BY USER in order to implement the localization functionalities.
 The method *localizationModule::respond()* implements a default parser for the basic rpc commands needed to communicate with a localization2DClient. 
 This is the list of default rpc commands:
* **[VOCAB_INAVIGATION] [VOCAB_NAV_GET_CURRENT_POS]** Returns the current position of the robot in the format: **[VOCAB_OK] <map_name> `<x> <y> <theta>`**
* **[VOCAB_INAVIGATION] [VOCAB_NAV_SET_INITIAL_POS] `<map_name> <x> <y> <theta>`** initialize the localization system with the specified coordinates.

