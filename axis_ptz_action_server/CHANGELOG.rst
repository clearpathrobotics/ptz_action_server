^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package axis_ptz_action_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Re-send the axis command if the camera isn't moving after 5s
* Add some additional logging, handle camera disconnections better
* Abort the action in-progress if the Axis camera stops publishing state messages
* Use format strings instead of .format
* Contributors: Chris Iverach-Brereton

0.1.1 (2023-04-11)
------------------
* Add limits files for the standard dome camera & Q62 series. Add a new launch argument to specify the limits file
* Contributors: Chris Iverach-Brereton

0.1.0 (2023-03-27)
------------------
* Initial release
* Contributors: Chris Iverach-Brereton
